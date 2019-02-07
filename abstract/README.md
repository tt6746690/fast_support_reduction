

###  Check `abstract.pdf`


```
# http://www.citethisforme.com/
#   1. set format to biblatex
#   2. copy -> paste to `references.bib`
#   3. open vim and type `:%s/\[break\]\s/\r/g`
```


### resources


+ a nice tutorial on [framebuffer internals](http://www.songho.ca/opengl/gl_fbo.html)
+ a nice tutorial on [rendering depth to texture](https://paroj.github.io/gltut/Positioning/Tut05%20Overlap%20and%20Depth%20Buffering.html)


---

#### Profiling 

##### 2d

```
build/support_reduction woody-cross 10 1 20 0.0001 1 1
```

![](2018-11-26-22-05-54.png)
+ first iteration



![](2018-11-26-23-04-03.png)
+ optimized overhang energy



### presentation 

+ overhang energy not printer specific but can be twisted to an advantage, in that we can use it to account for 
    + physics based material strength to support itself
    + make it balance i.e. center of mass