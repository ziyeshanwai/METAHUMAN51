if exist Build (rmdir Build /s/q)
mkdir Build
cd Build

cmake -G "Visual Studio 16 2019" -DCMAKE_INSTALL_PREFIX=Install -DDLIB_USE_MKL_FFT=0 -DDLIB_USE_BLAS=0 -DDLIB_USE_LAPACK=0 ../Source

"%_msbuild%msbuild.exe" dlib_project.sln /t:build /p:Configuration=Release

"%_msbuild%msbuild.exe" INSTALL.vcxproj /t:build /p:Configuration=Release

copy /y Install\lib\dlib19.21.99_release_64bit_msvc1927.lib ..\Lib\Win64\Release\dlib19.21.99_release_64bit_msvc1927.lib

if exist ..\Include\dlib (rmdir ..\Include\dlib /s/q)
xcopy /y/s/i Install\include\dlib ..\Include\dlib

cd ..
