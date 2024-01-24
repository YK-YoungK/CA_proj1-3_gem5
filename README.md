# Computer Architectures, Project 1-3 (2023 fall)

Implement branch predictor, cache replacement policy, and cache prefetcher on gem5. Course project 1-3 for Computer Architectures course in Peking University (2023 fall).

Project 2: Implement simple branch predictor (always taken) and perceptron branch predictor. See ```src/cpu/pred/mysimple.cc```, ```src/cpu/pred/mysimple.hh```, ```src/cpu/pred/myperceptron.cc```, ```src/cpu/pred/myperceptron.hh``` for more details.

Project 3: Implement clock cache replacement policy and perceptron-based cache prefetcher. For clock cache replacement policy, you can refer to ```mem/cache/replacement_policies/myclock_rp.cc``` and ```mem/cache/replacement_policies/myclock_rp.hh``` for more details. For perceptron-based cache prefetcher, I implement it based on signature path prefetcher. You can refer to ```mem/prefetch/spp_ppf.cc```, ```mem/prefetch/spp_ppf.hh```, ```mem/prefetch/sppv2_ppf.cc```, and ```mem/prefetch/sppv2_ppf.hh``` for more details.

Besides, each project has several simulation and evaluation tasks which are not listed above. You can refer to project instructions for more details. For how to compile and use gem5, you can refer to project manual.