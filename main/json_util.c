/* 极简 JSON 解析/构建（见 json_util.h）。 */

#include "json_util.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---------------- 字符串构建 ---------------- */

void sb_init(sbuf_t *sb, char *buf, size_t cap)
{
    sb->buf = buf;
    sb->cap = cap;
    sb->len = 0;
    sb->overflow = false;
    if (cap > 0) {
        buf[0] = '\0';
    }
}

void sb_puts(sbuf_t *sb, const char *s)
{
    if (s == NULL) {
        return;
    }
    size_t n = strlen(s);
    if (sb->overflow || sb->len + n + 1 > sb->cap) {
        sb->overflow = true;
        return;
    }
    memcpy(sb->buf + sb->len, s, n);
    sb->len += n;
    sb->buf[sb->len] = '\0';
}

void sb_printf(sbuf_t *sb, const char *fmt, ...)
{
    if (sb->overflow || sb->len + 1 >= sb->cap) {
        sb->overflow = true;
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(sb->buf + sb->len, sb->cap - sb->len, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= sb->cap - sb->len) {
        sb->overflow = true;
        return;
    }
    sb->len += (size_t)n;
}

void sb_json_strn(sbuf_t *sb, const char *s, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        unsigned char c = (unsigned char)s[i];
        switch (c) {
        case '"':
            sb_printf(sb, "\\\"");
            break;
        case '\\':
            sb_printf(sb, "\\\\");
            break;
        case '\n':
            sb_printf(sb, "\\n");
            break;
        case '\r':
            sb_printf(sb, "\\r");
            break;
        case '\t':
            sb_printf(sb, "\\t");
            break;
        default:
            if (c < 0x20) {
                sb_printf(sb, "\\u%04x", c);
            } else {
                sb_printf(sb, "%c", c);
            }
            break;
        }
        if (sb->overflow) {
            return;
        }
    }
}

void sb_json_str(sbuf_t *sb, const char *s)
{
    if (s == NULL) {
        sb_printf(sb, "null");
        return;
    }
    sb_json_strn(sb, s, strlen(s));
}

void sb_json_quoted(sbuf_t *sb, const char *s)
{
    if (s == NULL) {
        sb_printf(sb, "null");
        return;
    }
    sb_printf(sb, "\"");
    sb_json_strn(sb, s, strlen(s));
    sb_printf(sb, "\"");
}

/* ---------------- 解析 ---------------- */

typedef struct {
    char *buf;
    size_t len;
    size_t pos;
    json_pool_t *pool;
    const char *err;
    int depth;
} jparse_t;

static json_node_t *node_new(jparse_t *p)
{
    if (p->pool->used >= JSON_MAX_NODES) {
        p->err = "too many json nodes";
        return NULL;
    }
    json_node_t *n = &p->pool->nodes[p->pool->used++];
    memset(n, 0, sizeof(*n));
    return n;
}

static void skip_ws(jparse_t *p)
{
    while (p->pos < p->len) {
        char c = p->buf[p->pos];
        if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
            p->pos++;
        } else {
            break;
        }
    }
}

static bool peek(jparse_t *p, char c)
{
    skip_ws(p);
    return p->pos < p->len && p->buf[p->pos] == c;
}

static bool expect(jparse_t *p, char c)
{
    if (!peek(p, c)) {
        p->err = "unexpected character";
        return false;
    }
    p->pos++;
    return true;
}

static int hex4(const char *s)
{
    int v = 0;
    for (int i = 0; i < 4; i++) {
        char c = s[i];
        v <<= 4;
        if (c >= '0' && c <= '9') {
            v |= c - '0';
        } else if (c >= 'a' && c <= 'f') {
            v |= c - 'a' + 10;
        } else if (c >= 'A' && c <= 'F') {
            v |= c - 'A' + 10;
        } else {
            return -1;
        }
    }
    return v;
}

static size_t utf8_encode(unsigned cp, char *out)
{
    if (cp < 0x80) {
        out[0] = (char)cp;
        return 1;
    }
    if (cp < 0x800) {
        out[0] = (char)(0xC0 | (cp >> 6));
        out[1] = (char)(0x80 | (cp & 0x3F));
        return 2;
    }
    out[0] = (char)(0xE0 | (cp >> 12));
    out[1] = (char)(0x80 | ((cp >> 6) & 0x3F));
    out[2] = (char)(0x80 | (cp & 0x3F));
    return 3;
}

static bool parse_string_raw(jparse_t *p, const char **out, size_t *out_len)
{
    if (!expect(p, '"')) {
        return false;
    }
    size_t start = p->pos;
    size_t w = start;
    while (p->pos < p->len) {
        char c = p->buf[p->pos];
        if (c == '"') {
            p->buf[w] = '\0';
            p->pos++;
            *out = p->buf + start;
            *out_len = w - start;
            return true;
        }
        if (c == '\\') {
            p->pos++;
            if (p->pos >= p->len) {
                break;
            }
            char e = p->buf[p->pos++];
            char repl = 0;
            switch (e) {
            case '"': repl = '"'; break;
            case '\\': repl = '\\'; break;
            case '/': repl = '/'; break;
            case 'b': repl = '\b'; break;
            case 'f': repl = '\f'; break;
            case 'n': repl = '\n'; break;
            case 'r': repl = '\r'; break;
            case 't': repl = '\t'; break;
            case 'u': {
                if (p->pos + 4 > p->len) {
                    p->err = "bad \\u escape";
                    return false;
                }
                int cp = hex4(p->buf + p->pos);
                if (cp < 0) {
                    p->err = "bad \\u escape";
                    return false;
                }
                p->pos += 4;
                unsigned code = (unsigned)cp;
                if (code >= 0xD800 && code <= 0xDBFF && p->pos + 6 <= p->len &&
                    p->buf[p->pos] == '\\' && p->buf[p->pos + 1] == 'u') {
                    int lo = hex4(p->buf + p->pos + 2);
                    if (lo >= 0xDC00 && lo <= 0xDFFF) {
                        code = 0x10000u + ((code - 0xD800u) << 10) + ((unsigned)lo - 0xDC00u);
                        p->pos += 6;
                    }
                }
                w += utf8_encode(code, p->buf + w);
                continue;
            }
            default:
                p->err = "bad escape";
                return false;
            }
            p->buf[w++] = repl;
            continue;
        }
        p->buf[w++] = c;
        p->pos++;
    }
    p->err = "unterminated string";
    return false;
}

static json_node_t *parse_value(jparse_t *p);

static json_node_t *parse_object(jparse_t *p)
{
    json_node_t *obj = node_new(p);
    if (obj == NULL) {
        return NULL;
    }
    obj->type = JSON_OBJ;
    if (!expect(p, '{')) {
        return NULL;
    }
    json_node_t *tail = NULL;
    if (peek(p, '}')) {
        p->pos++;
        return obj;
    }
    for (;;) {
        const char *key = NULL;
        size_t key_len = 0;
        if (!parse_string_raw(p, &key, &key_len)) {
            return NULL;
        }
        if (!expect(p, ':')) {
            return NULL;
        }
        json_node_t *val = parse_value(p);
        if (val == NULL) {
            return NULL;
        }
        val->key = key;
        val->key_len = key_len;
        if (tail == NULL) {
            obj->child = val;
        } else {
            tail->next = val;
        }
        tail = val;
        if (peek(p, ',')) {
            p->pos++;
            continue;
        }
        if (!expect(p, '}')) {
            return NULL;
        }
        return obj;
    }
}

static json_node_t *parse_array(jparse_t *p)
{
    json_node_t *arr = node_new(p);
    if (arr == NULL) {
        return NULL;
    }
    arr->type = JSON_ARR;
    if (!expect(p, '[')) {
        return NULL;
    }
    json_node_t *tail = NULL;
    if (peek(p, ']')) {
        p->pos++;
        return arr;
    }
    for (;;) {
        json_node_t *val = parse_value(p);
        if (val == NULL) {
            return NULL;
        }
        if (tail == NULL) {
            arr->child = val;
        } else {
            tail->next = val;
        }
        tail = val;
        if (peek(p, ',')) {
            p->pos++;
            continue;
        }
        if (!expect(p, ']')) {
            return NULL;
        }
        return arr;
    }
}

static json_node_t *parse_value(jparse_t *p)
{
    skip_ws(p);
    if (p->pos >= p->len) {
        p->err = "unexpected end";
        return NULL;
    }
    if (p->depth++ > 8) {
        p->err = "too deep";
        return NULL;
    }
    char c = p->buf[p->pos];
    json_node_t *n = NULL;

    if (c == '{') {
        n = parse_object(p);
    } else if (c == '[') {
        n = parse_array(p);
    } else if (c == '"') {
        const char *s = NULL;
        size_t len = 0;
        if (parse_string_raw(p, &s, &len)) {
            n = node_new(p);
            if (n != NULL) {
                n->type = JSON_STR;
                n->str = s;
                n->str_len = len;
            }
        }
    } else if (strncmp(p->buf + p->pos, "true", 4) == 0) {
        p->pos += 4;
        n = node_new(p);
        if (n != NULL) {
            n->type = JSON_BOOL;
            n->boolean = true;
        }
    } else if (strncmp(p->buf + p->pos, "false", 5) == 0) {
        p->pos += 5;
        n = node_new(p);
        if (n != NULL) {
            n->type = JSON_BOOL;
            n->boolean = false;
        }
    } else if (strncmp(p->buf + p->pos, "null", 4) == 0) {
        p->pos += 4;
        n = node_new(p);
        if (n != NULL) {
            n->type = JSON_NULL;
        }
    } else {
        char *end = NULL;
        double v = strtod(p->buf + p->pos, &end);
        if (end == p->buf + p->pos) {
            p->err = "bad value";
            p->depth--;
            return NULL;
        }
        p->pos = (size_t)(end - p->buf);
        n = node_new(p);
        if (n != NULL) {
            n->type = JSON_NUM;
            n->num = v;
        }
    }
    p->depth--;
    return n;
}

json_node_t *json_parse(char *buf, size_t len, json_pool_t *pool, const char **err)
{
    jparse_t p = {.buf = buf, .len = len, .pos = 0, .pool = pool, .err = NULL, .depth = 0};
    pool->used = 0;
    json_node_t *root = parse_value(&p);
    if (root == NULL) {
        if (err != NULL) {
            *err = p.err ? p.err : "parse error";
        }
        return NULL;
    }
    skip_ws(&p);
    if (p.pos != p.len) {
        if (err != NULL) {
            *err = "trailing data";
        }
        return NULL;
    }
    return root;
}

/* ---------------- 访问器 ---------------- */

const json_node_t *json_obj_get(const json_node_t *obj, const char *key)
{
    if (obj == NULL || obj->type != JSON_OBJ) {
        return NULL;
    }
    size_t klen = strlen(key);
    for (const json_node_t *c = obj->child; c != NULL; c = c->next) {
        if (c->key != NULL && c->key_len == klen && memcmp(c->key, key, klen) == 0) {
            return c;
        }
    }
    return NULL;
}

const json_node_t *json_arr_get(const json_node_t *arr, size_t idx)
{
    if (arr == NULL || arr->type != JSON_ARR) {
        return NULL;
    }
    const json_node_t *c = arr->child;
    while (c != NULL && idx > 0) {
        c = c->next;
        idx--;
    }
    return c;
}

size_t json_arr_len(const json_node_t *arr)
{
    if (arr == NULL || arr->type != JSON_ARR) {
        return 0;
    }
    size_t n = 0;
    for (const json_node_t *c = arr->child; c != NULL; c = c->next) {
        n++;
    }
    return n;
}

const char *json_str(const json_node_t *v, size_t *len)
{
    if (v == NULL || v->type != JSON_STR) {
        return NULL;
    }
    if (len != NULL) {
        *len = v->str_len;
    }
    return v->str;
}

bool json_int(const json_node_t *v, long *out)
{
    if (v == NULL) {
        return false;
    }
    if (v->type == JSON_NUM) {
        *out = (long)v->num;
        return true;
    }
    if (v->type == JSON_BOOL) {
        *out = v->boolean ? 1 : 0;
        return true;
    }
    return false;
}

bool json_bool(const json_node_t *v, bool *out)
{
    if (v == NULL) {
        return false;
    }
    if (v->type == JSON_BOOL) {
        *out = v->boolean;
        return true;
    }
    if (v->type == JSON_NUM) {
        *out = v->num != 0;
        return true;
    }
    return false;
}

void json_serialize(const json_node_t *v, sbuf_t *sb)
{
    if (v == NULL) {
        sb_puts(sb, "null");
        return;
    }
    switch (v->type) {
    case JSON_NULL:
        sb_puts(sb, "null");
        break;
    case JSON_BOOL:
        sb_puts(sb, v->boolean ? "true" : "false");
        break;
    case JSON_NUM: {
        long l = (long)v->num;
        if ((double)l == v->num) {
            sb_printf(sb, "%ld", l);
        } else {
            sb_printf(sb, "%.6g", v->num);
        }
        break;
    }
    case JSON_STR:
        sb_json_quoted(sb, v->str);
        break;
    case JSON_ARR:
        sb_puts(sb, "[");
        for (const json_node_t *c = v->child; c != NULL; c = c->next) {
            if (c != v->child) {
                sb_puts(sb, ",");
            }
            json_serialize(c, sb);
        }
        sb_puts(sb, "]");
        break;
    case JSON_OBJ:
        sb_puts(sb, "{");
        for (const json_node_t *c = v->child; c != NULL; c = c->next) {
            if (c != v->child) {
                sb_puts(sb, ",");
            }
            sb_json_quoted(sb, c->key);
            sb_puts(sb, ":");
            json_serialize(c, sb);
        }
        sb_puts(sb, "}");
        break;
    }
}
