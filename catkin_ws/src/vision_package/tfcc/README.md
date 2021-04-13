This is a standalone project to test Tensorflow C++ (tensorflow_cc)

Open this folder in a new VSCode window.

Build tensorflow from source:   \
1 Clone tensorflow repo.    \
2 Install Bazel.    \
3 Configure tensorflow build with ./configure   \
4 Build tensorflow usign bazel. 
```
~/Downloads/tensorflow$ bazel build -c opt --config=monolithic --jobs=2 //tensorflow:libtensorflow_cc.so
```
5 Include files and directories in your project.    \
a) Incude files from repo   \
b) Include fies from cache. 