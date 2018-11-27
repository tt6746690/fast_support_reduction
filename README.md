

```
git clone --recursive git@github.com:tt6746690/fast_support_reduction.git

# profiling: minitrace + chrome tracing 
#       https://github.com/hrydgard/minitrace
#       https://www.chromium.org/developers/how-tos/trace-event-profiling-tool

mkdir build && cd build
cmake ..                        # debugging
cmake .. -DWITH_DEBUG=OFF       # 03!
cmake .. -DWITH_VISUALIZE=ON    # visualize correctness of energy computation
```