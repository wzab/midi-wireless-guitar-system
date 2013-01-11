/*
  The code below is published as PUBLIC DOMAIN as Wojciech M. Zabolotny
  ( wzab<at>ise.pw.edu.pl ) 2011.01.11
  No warranty of any kind is provided! Use at your own risk!
*/ 
#ifndef _BYTE_SELECT_H_
#define _BYTE_SELECT_
#define byte_select(v) \
  ((uint8_t *) &v)
__attribute__ ((always_inline))
static inline uint8_t uint32_b0(uint32_t v)
{
  union {
    uint32_t i;
    uint8_t b[4];
  } tmp;
  tmp.i = v;
  return tmp.b[0];
}
__attribute__ ((always_inline))
static inline uint8_t uint32_b1(uint32_t v)
{
  union {
    uint32_t i;
    uint8_t b[4];
  } tmp;
  tmp.i = v;
  return tmp.b[1];
}
__attribute__ ((always_inline))
static inline uint8_t uint32_b2(uint32_t v)
{
  union {
    uint32_t i;
    uint8_t b[4];
  } tmp;
  tmp.i = v;
  return tmp.b[2];
}
__attribute__ ((always_inline))
static inline uint8_t uint32_b3(uint32_t v)
{
  union {
    uint32_t i;
    uint8_t b[4];
  } tmp;
  tmp.i = v;
  return tmp.b[3];
}
#endif
