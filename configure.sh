cd bullet3
cmake .
make
cd ../zeromq-4.0.5
./configure --enable-static --disable-shared LDFLAGS=-lstdc++
make