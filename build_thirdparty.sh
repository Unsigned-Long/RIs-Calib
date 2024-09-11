# update submodules
echo "----------------------------------------------------"
echo "update submodules remotely, it may take some time..."
echo "----------------------------------------------------"
git submodule update --init --recursive
if [ $? -ne 0 ]; then
    echo "--------------------------------------------"
    echo "error occurs when updating submodules, exit!"
    echo "--------------------------------------------"
    exit
fi

# shellcheck disable=SC2046
RIS_CALIB_ROOT_PATH=$(cd $(dirname $0) || exit; pwd)
echo "the root path of 'RIs-Calib': ${RIS_CALIB_ROOT_PATH}"

# build tiny-viewer
echo "----------------------------------"
echo "build thirdparty: 'tiny-viewer'..."
echo "----------------------------------"

# shellcheck disable=SC2164
cd "${RIS_CALIB_ROOT_PATH}"/thirdparty/ctraj

chmod +x build_thirdparty.sh
./build_thirdparty.sh

# build ctraj
echo "----------------------------"
echo "build thirdparty: 'ctraj'..."
echo "----------------------------"

mkdir ${RIS_CALIB_ROOT_PATH}/thirdparty/ctraj-build
# shellcheck disable=SC2164
cd "${RIS_CALIB_ROOT_PATH}"/thirdparty/ctraj-build

cmake ../ctraj
echo current path: $PWD
echo "-----------------------"
echo "start making 'ctraj'..."
echo "-----------------------"
make -j8
cmake --install . --prefix ${RIS_CALIB_ROOT_PATH}/thirdparty/ctraj-install
