# WolfVision

WildWolf Team RoboMaster-2021 Visual &amp; Algorithm Group Code Framework.

## Enviroment

| name           | version                        |
|:--------------:|:------------------------------:|
| System         | [`Ubuntu 21.04`](https://discourse.ubuntu.com/t/hirsute-hippo-release-notes/19221) |
| OpenCV         | [`4.5.3`](https://github.com/opencv/opencv/releases/tag/4.5.2) |
| OpenCV_Contrib | [`4.5.3`](https://github.com/opencv/opencv_contrib/releases/tag/4.5.2) |
| CMake          | [`3.21.0`](https://cmake.org/) |
| GCC            | [`11.1.0`](https://ftp.gnu.org/gnu/gcc/gcc-11.1.0/) |
| GDB            | [`10.2`](https://www.gnu.org/software/gdb/download/) |
| MindVision-SDK | [`2.1.0`](http://mindvision.com.cn/rjxz/list_12.aspx) |

## Contribute

Project created using [Visual Studio Code](https://code.visualstudio.com/), required plugins listed below:

- C/C++ `ms-vscode.cpptools`
- CMake `twxs.cmake`
- CMake Tools `ms-vscode.cmake-tools`
- Visual Studio IntelliCode `visualstudioexptteam.vscodeintellicode`

Our project is here [WolfVision 2021 Project](https://github.com/orgs/wildwolf-team/projects/1).

### Guide

Before start coding, please finish project configuration first:

```shell
sudo bash scripts/autoconfig.sh
```

After configuration, enjoy coding follow [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html), linting by command:

```shell
bash scripts/cpplint-pre-commit.sh
```

### Commit Lint

- feat: new feature
- fix: modify the issue
- refactor: code refactoring
- docs: documentation changes
- style: code formatting changes
- test: test case modifications
- chore: other changes, such as build process, dependency management

## License

`MIT, Copyright WildWolf Team 2021`
