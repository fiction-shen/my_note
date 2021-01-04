## string 类 #include <string>

### string::substr

形式:s.substr(p,n) 返回一个string，包含字符串s中从p开始的n个字符的拷贝（p的默认值是0，n的默认值是s.size() - p，即不加参数会默认拷贝整个s）

```c++
1 int main()
 2 {
 3     string a;
 4     string s("123456789");
 5     
 6     a = s.substr(0,5);//拷贝字符串s中从第0位开始的长度为5的字符串
 7     cout << a << endl;//输出12345
 8     
 9     a=s.substr();//不加参数即默认拷贝整个字符串s
10     cout<<a<<endl;//输出123456789
11     
12     a=s.substr(4);//输出56789
13     cout<<a<<endl;//单个参数则默认拷贝从第4位开始至末尾的字符串
14 }
```

### memcpy

函数原型：void *memcpy(void str,const void *s,size_t n); 功能 c和c++使用的内存拷贝函数.从源s所指的内存地址的起始位置开始拷贝n个字节到目标str所指的内存地址的起始位置中 memcpy与strcpy有以下不同： 1.复制内容不同。strcpy复制字符串，而memcpy复制字符数组、整型、结构体、类等。 2.复制方法不同。strcpy遇到被复制字符串的'\0'结束，memspy由第三个参数决定复制的长度。

```c++
//将s中的字符串复制到字符数组str中   
#include<cstdio>  
#include<cstring>  
int main()  
{  
    char *s="abcd efg hi";  
    char str[20];  
    memcpy(str,s,strlen(s));  
    printf("%s\n",str);//输出abcd efg hi   
}
//将s中的下标为3个字符开始的连续8个字符复制到str中。  
#include<cstdio>  
#include<cstring>  
int main()  
{  
    char *s="abcdefg higk lmn";  
    char str[20];  
    memcpy(str,s+3,8);  
    str[8]='\0';   
    printf("%s\n",str);//输出defg hig   
}
//复制后覆盖原有的部分   
#include<cstdio>  
#include<cstring>  
int main()  
{  
    char s[20]="*******";  
    char str[20]="abcdefghigk";  
    memcpy(str,s,strlen(s));  
    printf("%s\n",str);//输出*******higk   
}
```

## int** a

可以这样子理解，

```c++
(int*)*a,指向(int*)型的指针。
```