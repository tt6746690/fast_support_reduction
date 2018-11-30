

```
git clone --recursive git@github.com:tt6746690/fast_support_reduction.git

# profiling: minitrace + chrome tracing 
#       https://github.com/hrydgard/minitrace
#       https://www.chromium.org/developers/how-tos/trace-event-profiling-tool

mkdir build && cd build

# debugging, or
cmake ..
# 03! or
cmake .. -DWITH_DEBUG=OFF
# visualize correctness of energy computation
cmake .. -DWITH_VISUALIZE=ON

make
```