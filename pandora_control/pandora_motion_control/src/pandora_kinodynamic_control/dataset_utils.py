import os
import sys
import pickle

## -- Set and hold RlData directory path -- ##
__path__ = os.path.dirname(__file__)
## ---------------------------------------- ##


## ------------------- Usefull Path Definitions -------------------------- ##
__rl_data_dir__ = os.path.expanduser("~/.rl_data")

__pandora_ws_dir__ = os.path.expanduser("~/pandora_ws")

__pandora_control_repo_dir__ = os.path.expanduser(
    __pandora_ws_dir__ + "/src/pandora_control")

__kinodynamic_control_dir__ = __pandora_control_repo_dir__ + \
    "/pandora_motion_control/src/pandora_kinodynamic_control"

__rl_data_repo_dir__ = os.path.expanduser(__kinodynamic_control_dir__ +
    "/tables")
## ---------------------------------------------------------------------- ##



def exec_system_cmd(cmd):
  try:
    os.system(cmd)
  except:
    e = sys.exc_info()[0]
    print e
    return False
  finally:
    return True


def create_rlData_dir():
    if os.path.exists(__rl_data_dir__):
        pass
    else:
        print "\033[1;32mCreating dir [%s]\033[0m" % __rl_data_dir__
        cmd = "mkdir %s" % __rl_data_dir__
        exec_system_cmd(cmd)


##
#  Create symbolic link to repository
#
def link_rlData_repo():
  if os.path.exists(__rl_data_dir__):
    # Symlinc || directory allready exists
    print "\033[1;33mDir [%s] already exists\033[0m" % __rl_data_dir__
    pass
  else:
    cmd = "ln -s %s %s" % (__rl_data_repo_dir__, __rl_data_dir__)
    print "\033[1;32mCreating Symlink from %s to %s" \
        % (__rl_data_dir__, __rl_data_repo_dir__)
    exec_system_cmd(cmd)


##
#  Copies from local data dir to repo dir
##
def add_to_repo():
    if os.path.exists(__rl_data_repo_dir__):
        cmd = "cp  %s/* %s/" %(__rl_data_dir__, __rl_data_repo_dir__)
        exec_system_cmd(cmd)
        return True
    return False

def push_data_origin():
  branch = 'origin/master'
  commit_msg = 'Updated RL data'
  cmd = "cd %s/.. && git add -A . && git commit -m %s" + \
      "&& git push %" % (__rl_data_repo_dir__, commit_msg, branch)
  exec_system_cmd(cmd)

def loadData(filename):
    filepath = __rl_data_dir__+"/"+filename
    if os.path.exists(filepath) and os.path.isfile(filepath):
        fileObject = open(filepath, 'r')
        _av_table = pickle.load(fileObject)
        fileObject.close()
        return [_av_table,True]
    else:
        return [0,False]

def storeData(data,filename):
    create_rlData_dir()
    filepath = __rl_data_dir__+"/"+filename
    fileObject = open(filepath ,'w')
    pickle.dump(data,fileObject)
    fileObject.close()
