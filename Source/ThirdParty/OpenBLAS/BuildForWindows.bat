if exist Build (rmdir Build /s/q)
mkdir Build
cd Build

cmake -G "Visual Studio 16 2019" ../Source

"%_msbuild%msbuild.exe" openblas.vcxproj /t:build /p:Configuration=Release

copy /y lib\RELEASE\openblas.lib ..\Lib\Win64\Release\openblas.lib

cd ..
