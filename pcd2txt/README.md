# pcd2txt
pcd转txt。
## Output file structure
```
  x      y      z    intensity       # position (not printable)
-0.1    0.1    0.1    0.0741
-0.2    0.2    0.2    0.0762
...     ...    ...    ...
```
## Compile
```
cd pcd2txt
mkdir build
cd build
cmake ..
make
```
## Run

```
./pcd2txt <pcd file> -o <output dir>
```


