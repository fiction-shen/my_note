# sprintf函数用法详解

函数名: sprintf
功    能: 送格式化输出到字符串中
用    法: int sprintf(char *string, char *farmat [,argument,...]);

说    明：函数sprintf()的用法和printf()函数一样，只是sprintf()函数给出第一个参数string(一般为字符数组）

该函数包含在stdio.h的头文件中。

sprintf 最常见的应用之一莫过于把整数打印到字符串中，所以，spritnf 在大多数场合可以替代itoa。如：

```c++
//把整数123 打印成一个字符串保存在s 中
sprintf(s, "%d", 123);     //产生"123"

//可以指定宽度，不足的左边补空格：
sprintf(s, "%8d%8d", 123, 4567); //产生：" 123 4567"

//当然也可以左对齐：
sprintf(s, "%-8d%8d", 123, 4567); //产生："123 4567"

//也可以按照16 进制打印：
sprintf(s, "%8x", 4567); //小写16 进制，宽度占8 个位置，右对齐
sprintf(s, "%-8X", 4568); //大写16 进制，宽度占8 个位置，左对齐
```



# fprintf函数用法详解

**fprintf()用于文件操作**

```c++
#include <stdio.h>
int fprintf( FILE *stream, const char *format, ... );
```

fprintf()函数根据指定的format(格式)发送信息(参数)到由stream(流)指定的文件.因此fprintf()可以使得信息输出到指定的文件.比如

```c++
    char name[20] = "Mary";
    FILE *out;
    out = fopen( "output.txt", "w" );
    if( out != NULL )
    fprintf( out, "Hello %s\n", name );
```

fprintf()的返回值是输出的字符数,发生错误时返回一个负值. 

```c++

#include <stdio.h>
#include<stdlib.h> 
FILE *stream;
void main( void )
{
int i = 10;
double fp = 1.5;
char s[] = "this is a string";
char c = '\n';
stream = fopen( "fprintf.out", "w" );
fprintf( stream, "%s%c", s, c );
fprintf( stream, "%d\n", i );
fprintf( stream, "%f\n", fp );
fclose( stream );
system( "type fprintf.out" );

```

