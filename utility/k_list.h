/****************************************************************************
 * Copyright (c) [2019] [core.zhang@outlook.com]                            *
 * [Software Name] is licensed under Mulan PSL v2.                          *
 * You can use this software according to the terms and conditions of       *
 * the Mulan PSL v2.                                                        *
 * You may obtain a copy of Mulan PSL v2 at:                                *
 *          http://license.coscl.org.cn/MulanPSL2                           *
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF     *
 * ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO        *
 * NON-INFRINGEMENT, MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.       *
 * See the Mulan PSL v2 for more details.                                   *
 *                                                                          *
 ***************************************************************************/
#ifndef _K_LIST_H
#define _K_LIST_H
#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \brief: 链表操作头文件，由linux内核 list.h头文件简化修改而来
 *          仅保留了常用的一些宏函数
 *
 *  注意：链表的头节点只作为控制节点
 */

#define STATIC_INLINE   static inline

struct k_list_head {
    struct k_list_head *next;
    struct k_list_head *prev;
};

#define K_LIST_HEAD_INIT(name) {&(name), &(name)}

#define K_LIST_HEAD(name) \
    struct k_list_head name = K_LIST_HEAD_INIT(name)

#define K_INIT_LIST_HEAD(list)  k_list_init(list)

STATIC_INLINE void k_list_init(struct k_list_head *list)
{
    list->next = list;
    list->prev = list;
}

/*! 将new对象插入到 pre对象和next对象的中间*/
STATIC_INLINE void __list_add(struct k_list_head *new,
                              struct k_list_head *pre,
                              struct k_list_head *next)
{
    next->prev = new;
    new->next = next;
    new->prev = pre;
    pre->next = new;
}

/*! 将new对象插入到head节点后面*/
STATIC_INLINE void k_list_add(struct k_list_head *new,
                              struct k_list_head *head)
{
    __list_add(new, head, head->next);
}

/*! 将new对象插入到head节点前面*/
STATIC_INLINE void k_list_add_tail(struct k_list_head *new,
                                   struct k_list_head *head)
{
    __list_add(new, head->prev, head);
}

/**
 *  \brief: 删除一个节点，前提是pre和next已存在
 */
STATIC_INLINE void __list_del(struct k_list_head *pre,
                              struct k_list_head *next)
{
    next->prev = pre;
    pre->next = next;
}

STATIC_INLINE void __list_del_entry(struct k_list_head *entry)
{
    __list_del(entry->prev, entry->next);
}

STATIC_INLINE void k_list_del(struct k_list_head *entry)
{
    __list_del(entry->prev, entry->next);
}

/**
 *  \brief: 从链表中删除条目并对其进行初始化
 *  entry:  从链表中删除的条目
 */
STATIC_INLINE void k_list_del_init(struct k_list_head *entry)
{
    __list_del_entry(entry);
    k_list_init(entry);
}

/**
 *  \brief: 从链表中删除一个条目，并插入到另外一个链表节点的前面
 *  entry:  从链表中删除的条目
 *  head:   被插入的链表条目头
 */
STATIC_INLINE void k_list_move(struct k_list_head *list,
                               struct k_list_head *head)
{
    __list_del_entry(list);
    k_list_add(list, head);
}

/**
 *  \brief: 从链表中删除一个条目，并插入到另外一个链表节点的尾部
 *  list:   从链表中删除的条目
 *  head:   被插入的链表条目头
 */
STATIC_INLINE void k_list_move_tail(struct k_list_head *list,
                                    struct k_list_head *head)
{
    __list_del_entry(list);
    k_list_add_tail(list, head);
}

/**
 *  \brief: 判断一个节点是否为链表中的最后一个节点
 *  list:   链表头
 *  head:   链表某一节点
 */
STATIC_INLINE int k_list_is_last(const struct k_list_head *list,
                                 const struct k_list_head *head)
{
    return list->next == head;
}

/**
 *  \brief: 判断一个链表是否为空链表
 *  head:   链表头
 */
STATIC_INLINE int k_list_empty(const struct k_list_head *head)
{
    return head->next == head;
}

/**
 *  \brief: 将链表list插入到另一个链表的某个节点之前
 *          第一个链表 [h] - A - B - C 第二个链表 [h] - c - d - e
 *          插入到C之前的结果 [h] - A - B - c - d - e - C
 *  list:   链表头
 */
STATIC_INLINE void __list_splice(const struct k_list_head *list,
                                 struct k_list_head *prev,
                                 struct k_list_head *next)
{
    struct k_list_head *first = list->next; // 链表第一个节点（不包括头节点）
    struct k_list_head *last = list->prev;  // 链表最后一个节点

    first->prev = prev; // c -> B
    prev->next = first; // B - > c

    last->next = next; // e -> C
    next->prev = last; // C -> e
}

/**
 *  \brief: 拼接两个链表, 头插法
 *  list:   链表头节点
 *  head：  被插入的链表节点
 */
static inline void k_list_splice(const struct k_list_head *list,
                                 struct k_list_head *head)
{
    if (!k_list_empty(list))
        __list_splice(list, head, head->next);
}

/**
 *  \brief: 拼接两个链表，尾插法
 *  list:   链表头节点
 *  head：  被插入的链表节点
 */
static inline void k_list_splice_tail(struct k_list_head *list,
                                      struct k_list_head *head)
{
    if (!k_list_empty(list))
        __list_splice(list, head->prev, head);
}

static inline void k_list_splice_init(struct k_list_head *list,
                                      struct k_list_head *head)
{
    if (!k_list_empty(list)) {
        __list_splice(list, head, head->next);
        k_list_init(list);
    }
}

static inline void k_list_splice_tail_init(struct k_list_head *list,
                                           struct k_list_head *head)
{
    if (!k_list_empty(list)) {
        __list_splice(list, head->prev, head);
        k_list_init(list);
    }
}

#define k_offset_of(type, member)                       \
        ((char *)&((type *)0)->member)

#define k_container_of(ptr, type, member)               \
        ((type *)((char *)(ptr) - k_offset_of(type, member)))

#define k_list_entry(ptr, type, member)                 \
        k_container_of(ptr, type, member)

#define k_list_first_entry(ptr, type, member)           \
        k_list_entry((ptr)->next, type, member)

#define k_list_last_entry(ptr, type, member)            \
        k_list_entry((ptr)->prev, type, member)

#define k_list_first_entry_or_null(ptr, type, member)   \
       (!k_list_empty(ptr) ? k_list_first_entry(ptr, type, member) : NULL)

/**
 *  \brief: 迭代一个链表
 *  pos:    struct k_list_head 类型节点，作为循环的终止光标
 *  head:   链表的头节点
 *  Note:   不能在这个循环中删除节点，因为 list_del 和 list_del_init
 *          会修改节点的指向
 */
#define k_list_for_each(pos, head)                          \
        for (pos = (head)->next; pos != (head); pos = pos->next)

#define k_list_for_each_prev(pos, head)                     \
        for (pos = (head)->prev; pos != (head); pos = pos->prev)

/**
 *  \brief: 迭代一个链表
 *  pos:    struct k_list_head 类型节点，作为循环的终止光标
 *  n:      struct k_list_head 类型节点指针，临时处理变量
 *  head:   链表的头节点
 */
#define k_list_for_each_safe(pos, n, head)                  \
        for (pos = (head)->next, n = pos->next; pos != head;\
             pos = n, n = pos->next)

#define k_list_for_each_prev_safe(pos, n, head)             \
        for (pos = (head)->prev, n = pos->prev; pos != head;\
             pos = n, n = pos->prev)

/*! 嵌入式平台大多不支持 typeof, 用户需要手动指定类型 */
#ifndef UNSUPPORT_TYPEOF

#define k_list_for_each_entry(pos, head, member)                        \
    for (pos = k_list_entry((head)->next, typeof(*pos), member);        \
         &pos->member != (head);                                        \
         pos = k_list_entry(pos->member.next, typeof(*pos), member))

#define k_list_for_each_entry_reverse(pos, head, member)                \
    for (pos = k_list_entry((head)->prev, typeof(*pos), member);        \
         &pos->member != (head);                                        \
         pos = k_list_entry(pos->member.prev, typeof(*pos), member))

#define k_list_for_each_entry_safe(pos, n, head, member)                \
    for (pos = k_list_entry((head)->next, typeof(*pos), member),        \
         n = k_list_entry(pos->member.next, typeof(*pos), member);      \
         &pos->member != (head);                                        \
         pos = n, n = k_list_entry(n->member.next, typeof(*n), member))

#define k_list_for_each_entry_safe_reverse(pos, n, head, member)        \
    for (pos = k_list_entry((head)->prev, typeof(*pos), member),        \
         n = k_list_entry(pos->member.prev, typeof(*pos), member);      \
         &pos->member != (head);                                        \
         pos = n, n = k_list_entry(n->member.prev, typeof(*n), member))

#else

#define k_list_for_each_entry(pos, type, head, member)				    \
    for (pos = k_list_entry((head)->next, type, member);	            \
         &pos->member != (head); 	                                    \
         pos = k_list_entry(pos->member.next, type, member))

#define k_list_for_each_entry_reverse(pos, type, head, member)		    \
    for (pos = k_list_entry((head)->prev, type, member);	            \
         &pos->member != (head); 	                                    \
         pos = k_list_entry(pos->member.prev, type, member))

#define k_list_for_each_entry_safe(pos, n, type, head, member)			\
    for (pos = k_list_entry((head)->next, type, member),	            \
         n = k_list_entry(pos->member.next, type, member);	            \
         &pos->member != (head); 					                    \
         pos = n, n = k_list_entry(n->member.next, type, member))

#define k_list_for_each_entry_safe_reverse(pos, n, type, head, member)  \
    for (pos = k_list_entry((head)->prev, type, member),	            \
         n = k_list_entry(pos->member.prev, type, member);	            \
         &pos->member != (head); 					                    \
         pos = n, n = k_list_entry(n->member.prev, type, member))

#endif

#ifdef __cplusplus
}
#endif
#endif

/*************************** End of file ****************************/
