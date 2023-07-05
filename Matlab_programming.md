# MATLAB程序设计导论
by Eugeniy E.Mikhailov

## 简介
定理：不可能证明一个足够复杂的计算机程序的正确性
因此，在编程时就应当有意识地寻找测试用例
有意识地发觉自己调试程序的能力

高级语言、低级语言
编译型语言、解释性语言

MATLAB是一种交互式语言

### 计算机中的数字表示与问题
1. 离散化——位数有限
2. 二进制——溢出错误
   ——使用具有相似数量级的数据时，不要相信结果的最小有效数字

## 最重要的功能：查看文档
``` MATLAB
docsearch word
% 在帮助文档中搜索检索词

help name
% 直接在命令窗口输出相应函数和方法的简短帮助

doc name
% 在帮助浏览器中显示函数或方法的引用界面
```

```
exp() % 以e为底的指数
```

```
plot(x, y,'x') %使用两个矩阵和指定符号创建二维图表
xlabel('x (radians)')
ylabel('y sin(x)')
title('10 points')
set(gca, 'FontSize', 24) %调用句柄设置字号
```


```
print('-dpng', '-r100', 'sin_of_x') 
% 将图像作为png格式保存，r分辨率为800*600
```