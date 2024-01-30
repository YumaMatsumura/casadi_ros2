# casadi_ros2

## casadiのインストールからビルドまで

```bash
$ sudo apt update
$ sudo apt install build-essential
$ sudo apt install coinor-libipopt-dev

$ sudo apt install gfortran liblapack-dev pkg-config --install-recommends
$ sudo apt install swig
$ git clone https://github.com/casadi/casadi.git -b master casadi
$ cd casadi
$ mkdir build
$ cd build
$ cmake -DWITH_PYTHON=ON -DWITH_IPOPT=ON -DWITH_OPENMP=ON -DWITH_THREAD=ON ..
$ make
$ sudo make install
```

## 本パッケージのビルド

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/YumaMatsumura/casadi_ros2.git
cd ~/ros2_ws
colcon build
```

## 非線形最適化問題を解くros2ノードを実行

```bash
ros2 run casadi_ros2 nonlinear_programs
```

## 参考

- [https://web.casadi.org/](https://web.casadi.org/)
- [https://github.com/zehuilu/Tutorial-on-CasADi-with-CPP/tree/main](https://github.com/zehuilu/Tutorial-on-CasADi-with-CPP/tree/main)
- [https://github.com/casadi/casadi](https://github.com/casadi/casadi)
