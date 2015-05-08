# VRPD

**To compile:**

Make sure you have installed cmake, and run the scripts:
~~~
cmake cmakelist
make
~~~
The execuable file "vrpd" is generated in ./bin

**To use:**

The [vrpd]( chrome://not-a-link) takes a couple of arguments. For more details, please run
~~~
./bin/vrpd -?
~~~

---

> ~~简书居然没有官方 Markdown 教程，我来~~
原来官方是有的。。。[献给写作者的 Markdown 新手指南][official_md_guide]
不过我这个更简。而且还有独门秘籍。

首先，“Markdown 其实很简单。在简书上学习 Markdown 最方便。”

[official_md_guide]: http://jianshu.io/p/q81RER



---

# 1. 标题

为了获得上面的 “`1. 标题`”， 在 Markdown 编辑器里输入：

~~~
# 1. 标题
~~~

“`#`” 后最好加个空格。除此之外，还有 5 级标题，依次有不同的字体大小，即

~~~
## 二级标题
### 三级标题
#### 四级标题
##### 五级标题
###### 六级标题
~~~

这样就有：

## 二级标题
### 三级标题
#### 四级标题
##### 五级标题
###### 六级标题



---

# 2. 加粗，斜体

最常用的强调方式，那就是 **加粗** 了，你得这样：

~~~
最常用的强调方式，那就是 **加粗** 了，你得这样：
~~~

通常我喜欢在 “`**加粗的部分**`” 旁边各加一个空格，当然你也可以不这样。
斜体则多用在于书名，比如：我从来没看过 *Jane Eyre*

~~~
斜体则多用在于书名，比如：我从来没看过 *Jane Eyre*
~~~

但中文的斜体我觉得真是不美，像：《*简 · 爱*》，一般还是别用了。



---

# 3. 层次

比如写个读书笔记，你得

#### 第一章

1. 第一节
* 第二节(你不用敲 "2"，自动就有了）
    * 第一小节（推荐每层次缩进四个空格）
        * 小小节 1
        * 小小节 2
    * 第二小节

也不难：

~~~
#### 第一章

1. 第一节
* 第二节(你不用敲 "2"，自动就有了）
    * 第一小节（推荐每层次缩进四个空格）
        * 小小节 1
        * 小小节 2
    * 第二小节
~~~

“`*`” 后面要加空格，这是必须的，除了 `*`，还可以使用 `+` 或者 `-`。

如果格式出现问题，多加个空行，一般就好了。



---

# 4. 链接，图片

你：我没读过 *Jane Eyre*
我：以后别跟我说话！
你：。。。
我：我也没读过，但是， [***Jane Eyre***](http://book.douban.com/subject/1141406/) is not just ***Jane Eyre***
![](http://img3.douban.com/mpic/s1108264.jpg)

~~~
我：我也没读过，但是， [***Jane Eyre***](http://book.douban.com/subject/1141406/) is not just ***Jane Eyre***
![](http://img3.douban.com/mpic/s1108264.jpg)
~~~



---

# [5. 其他][null-link]

你可能还没注意到本文每部分之间的分割线和 `其他` 的链接其实没有链接
我爱 `分割线`， 我爱 [**链接**][null-link]，哪怕它只有颜色~

[null-link]: chrome://not-a-link

~~~
---

# [5. 其他][null-link]

你可能还没注意到本文每部分之间的分割线和 `其他` 的链接其实没有链接
我爱 `分割线`， 我爱 [**链接**][null-link]，哪怕它只有颜色~

[null-link]: chrome://not-a-link
~~~

“`---`” 的上下最好各空一行

---

**P.S.** 补充一种高端的链接: [鼠标移过来，**先别单击** ~][hover]
[hover]: http://www.google.com.sg "Google Sg 更快，更好用。好，现在单击吧"

代码如下：

~~~
**P.S.** 补充一种高端的链接: [鼠标移过来，**先别单击** ~][hover]
[hover]: http://www.google.com.sg "Google Sg 更快，更好用。好，现在单击吧"
~~~

（可惜 Google 被墙了）

**P.P.S.** 图片链接：(点击图片可跳转）
[![][jane-eyre-pic]][jane-eyre-douban]
[jane-eyre-pic]: http://img3.douban.com/mpic/s1108264.jpg
[jane-eyre-douban]: http://book.douban.com/subject/1141406/

代码如下： 
```
[![][jane-eyre-pic]][jane-eyre-douban]
[jane-eyre-pic]: http://img3.douban.com/mpic/s1108264.jpg
[jane-eyre-douban]: http://book.douban.com/subject/1141406/
```
