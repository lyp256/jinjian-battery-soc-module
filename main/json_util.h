#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 极简 JSON 解析/构建工具（RPC 用，无动态内存）。
 * 解析：就地反转义字符串，节点来自调用方提供的静态池。 */

#define JSON_MAX_NODES 160

typedef enum {
    JSON_NULL = 0,
    JSON_BOOL,
    JSON_NUM,
    JSON_STR,
    JSON_ARR,
    JSON_OBJ,
} json_type_t;

typedef struct json_node {
    json_type_t type;
    bool boolean;
    double num;
    const char *str;     /* JSON_STR：指向源缓冲 */
    size_t str_len;
    const char *key;     /* 对象成员的键（数组元素为 NULL） */
    size_t key_len;
    struct json_node *child; /* 数组首元素 / 对象首成员的值 */
    struct json_node *next;  /* 同级下一个 */
} json_node_t;

typedef struct {
    json_node_t nodes[JSON_MAX_NODES];
    int used;
} json_pool_t;

/* 解析 JSON（buf 会被就地修改）。失败返回 NULL 并给出 err。 */
json_node_t *json_parse(char *buf, size_t len, json_pool_t *pool, const char **err);

/* 对象取成员 / 数组取元素 */
const json_node_t *json_obj_get(const json_node_t *obj, const char *key);
const json_node_t *json_arr_get(const json_node_t *arr, size_t idx);
size_t json_arr_len(const json_node_t *arr);

/* 取值助手 */
const char *json_str(const json_node_t *v, size_t *len);
bool json_int(const json_node_t *v, long *out);
bool json_bool(const json_node_t *v, bool *out);

/* 字符串构建（自动截断并置 overflow） */
typedef struct {
    char *buf;
    size_t cap;
    size_t len;
    bool overflow;
} sbuf_t;

void sb_init(sbuf_t *sb, char *buf, size_t cap);
void sb_puts(sbuf_t *sb, const char *s);
void sb_printf(sbuf_t *sb, const char *fmt, ...);
void sb_json_str(sbuf_t *sb, const char *s);      /* 带转义的 JSON 字符串 */
void sb_json_strn(sbuf_t *sb, const char *s, size_t len);
void sb_json_quoted(sbuf_t *sb, const char *s);   /* "\"...\"" 形式 */

/* 把已解析的 JSON 重新序列化（写回 sbuf） */
void json_serialize(const json_node_t *v, sbuf_t *sb);
