# aruco-detect

Setting up Virtual Enviroment (Necessary)
- Open terminal and type "python  -m venv ./venv" (You may need to type python3 instead of python)
- You should see a new folder called venv appear in your directory
- You should be able to find a file called activate/activate.bat
-   If you are on MAC or using a git Bash terminal
    - Type source ./venv/<path to "activate" file>
-   If you are using cmd or PowerShell
    - Type .\venv\<path to "activate" file>
You should get some indicator that says (venv in your terminal),
you can often check to make sure it worked by typing pip -V and checking that the output file path corresponds
to your currect directory

Now type in any dependencies found below

dependencies
- pip install opencv-contrib-python