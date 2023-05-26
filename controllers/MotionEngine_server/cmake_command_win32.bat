echo vcpkg root directory is %VCPKG_ROOT%
cmake .. "-DCMAKE_BUILD_TYPE=Release" "-DVCPKG_TARGET_TRIPLET=x64-windows" "-DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%/scripts/buildsystems/vcpkg.cmake"
cmake --build .