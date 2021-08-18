sudo apt-get install libgtest-dev -y
sudo apt-get install cmake -y

cd /usr/src/gtest

sudo cmake CMakeLists.txt

sudo make

cd lib/
sudo cp *.a /usr/lib

sudo mkdir /usr/local/lib/gtest/
sudo ln -s /usr/lib/libgtest.a /usr/local/lib/gtest/libgtest.a
sudo ln -s /usr/lib/libgtest_main.a /usr/local/lib/gtest/libgtest_main.a