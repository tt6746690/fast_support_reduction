
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
# look_moon
build/support_reduction bb-bunny 10 100 20 45 60 0 0.4 0 0

# the new .tgf, good result! one arm up, one arm down
build/support_reduction bb-bunny 5 50 50 45 60 50 0.5 1 0

# both arm up 
build/support_reduction bb-bunny 5 50 50 45 90 20 30 1 0

# farsee
build/support_reduction bb-bunny 5 120 200 50 90 20 40 0.5 0
```


### presentation 

+ overhang energy not printer specific but can be twisted to an advantage, in that we can use it to account for 
    + physics based material strength to support itself
    + make it balance i.e. center of mass