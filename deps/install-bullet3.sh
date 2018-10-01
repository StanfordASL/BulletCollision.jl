git clone https://github.com/bulletphysics/bullet3.git $HOME/bullet3

pushd $HOME/bullet3
mkdir bullet-build
cd bullet-build
cmake .. -G "Unix Makefiles" -DINSTALL_LIBS=ON -DBUILD_SHARED_LIBS=ON
make -j $(nproc --all)
sudo make install
cd ..

BULLET3_DIRECTORY=$(cd `dirname "$0"` && pwd)
mycmd="$BULLET3_DIRECTORY"
echo "export BULLET3=$mycmd" >> ~/.bashrc
source ~/.bashrc

popd
