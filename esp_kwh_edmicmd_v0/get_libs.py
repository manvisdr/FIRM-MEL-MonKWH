import os

Import("env")

LIBS_DIR = env.subst("$PROJECT_DIR/lib")

if not os.path.exists(LIBS_DIR):
   print("Cloning manvisdr/lib_MonKwh repo ... ")
   env.Execute("git clone --recurse-submodules --depth 100 https://github.com/manvisdr/lib_MonKwh $PROJECT_DIR/lib")
else:
   print("Checking for manvisdr/lib_MonKwh repo updates ... ")
   env.Execute("git --work-tree=$PROJECT_DIR\lib --git-dir=$PROJECT_DIR\lib\.git pull --progress -v --no-rebase origin --depth 100")
   # env.Execute("git --work-tree=$PROJECT_DIR\lib --git-dir=$PROJECT_DIR\lib\.git pull origin master --depth 100")
   # env.Execute("git --recurse-submodules --work-tree=$PROJECT_DIR\lib --git-dir=$PROJECT_DIR\lib\.git pull origin master --depth 100")