## Prerequisites   

- sudo apt install libeigen3-dev 
- sudo apt install clang-format 


## Building 

- Clone the repository

```
mkdir build/ && cd build/ 
cmake .. 
make 
```

## Contributing  

- Use clang-format with provided clang-format file,  
From the root of the repository, run the following command to format all files
```
find . -regex '.*\.\(cpp\|hpp\|cc\|cxx\)' -exec clang-format -style=file -i {}  \;  
``` 





