
#### Profiling 

##### 2d

```
build/support_reduction woody-cross 10 1 20 0.0001 1 1
```

![](2018-11-26-22-05-54.png)
+ first iteration



![](2018-11-26-23-04-03.png)
+ optimized overhang energy


##### 3d

```
build/support_reduction hand 10 1 20 0.0001 1 1
```


```
build/support_reduction hand 1 1 1 45 20 100 20000 10 0
build/support_reduction hand 1 10 10 45 20 100 20000 10 0
# fX: 13.6429
# ok, but not perfect

build/support_reduction hand 1 20 20 45 20 100 20000 10 0
# final fX: 9.06522
# visualy worse than previous

build/support_reduction hand 1 40 50 45 20 100 20000 10 0
final fX: 13.6429



build/support_reduction bb-bunny 4 1 1 45 20 100 20000 100 0
```