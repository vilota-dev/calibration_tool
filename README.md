## Trouble shooting stuff
- NFDE library changed from nativefiledialog-extended to ..._extended to be valid for cmake

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

### Get Started

Change directories and Build

```
cd src
make
```

Run

```
./example_app
```

Delete build files

```
make clean
```
