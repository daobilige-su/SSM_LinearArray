echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 1

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 1

cd ../../libgp

echo "Configuring and building Thirdparty/libgp ..."
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 1

cd ../../../

echo "Uncompress vocabulary ..."

cd ROS/SSM_LinearArray/Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ../../../

echo "Configuring and building SSM_LinearArray ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 1
cd ..

echo "Configuring and building SSM_LinearArray ROS package ..."

cd ROS/SSM_LinearArray/
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 1
cd ../../../
