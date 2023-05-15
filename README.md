## Todo
- Make into dockspace
- [Turn off unncessary builds like the building the tests](https://gist.github.com/UnaNancyOwen/acfc71de5b157d2ba22c090b420030e4)

## Trouble shooting stuff
- NFDE library changed from nativefiledialog-extended to ..._extended to be valid for cmake
- #define IMGUI_DEFINE_MATH_OPERATORS
- I added this shit ^ before the include imgui.h in the immvision. Pushed to forked version of immvision, check under src_all_in_one.
- Segmentation faults -> More often then not it's got something to do with uninitialized vectors and ImGui library.

## Setup
```
mkdir build
cd build/
cmake ..
cmake --build . -- -j
```


### Prerequisites 

C++
```
sudo apt-get install gcc g++
```

Libraries for graphics (Linux)

```
sudo apt-get install libglfw3 libglfw3-dev xorg xorg-dev
```

Libraries for graphics (Mac OS)
```
brew install glfw
```

## Preprocessor Macros
`PROJECT_ROOT`: Points to root of project / where cmake was invoked.

## Submodules
- [DearImGui](https://github.com/ocornut/imgui/tree/031148dc56d70158b3ad84d9be95b04bb3f5baaf)
- [ImPlot](https://github.com/epezent/implot/tree/18758e237e8906a97ddf42de1e75793526f30ce9)
- Basalt-headers
- Cereal
- Eigen
- Glad
- imgui
- immvision
- rosbag
- spdlog
- 

## Other relevant repos
[How to use visualizer in dear imgui application #5920](https://github.com/isl-org/Open3D/issues/5920)
https://github.com/vilota-dev/calibration-experiment
https://github.com/SMRT-AIST/interactive_slam/tree/master
[Visualize point cloud](https://stackoverflow.com/questions/10106288/pcl-visualize-a-point-cloud)
[Tanagram](https://www.tangramvision.com/resources/depth-sensor-visualizer)
