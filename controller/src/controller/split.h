#ifndef split_h
#define split_h

#include <Arduino.h>

//文字列分割関数(分割したい文字列,区切り文字,分割した文字列を代入するString配列)
int split(String data, char delimiter, String *dst);

#endif
