echo "Modify the build.sh if you want to use vcpkg ..."
echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release #-DCMAKE_TOOLCHAIN_FILE=E:/vcpkg-for-developer/scripts/buildsystems/vcpkg.cmake
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release #-DCMAKE_TOOLCHAIN_FILE=E:/vcpkg-for-developer/scripts/buildsystems/vcpkg.cmake
make -j

cd ../../../

echo "Configuring and building ORB_SLAM3 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release #-DCMAKE_TOOLCHAIN_FILE=E:/vcpkg-for-developer/scripts/buildsystems/vcpkg.cmake
make -j
