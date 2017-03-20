# Filename: .bashrc
# Description: Sources in on the class MASTER version for settings information
# 
# Please (DO NOT) edit this file unless you are sure of what you are doing.
# This file and other dotfiles have been written to work with each other.
# Any change that you are not sure off can break things in an unpredicatable
# ways.

# Set the Class MASTER variable and sources the class master version of .cshrc
if [ -z "$MASTER" ]; then
    export MASTER=${USER%-*}
    export MASTERDIR=`eval "echo ~$MASTER"`
fi

if [ -e $MASTERDIR/adm/class.zshenv ]; then
    source $MASTERDIR/adm/class.zshenv
fi

# Resetting umask to make sure things don't break
umask 022

# set path for class master
export PATH=/share/b/grading/sbin:$MASTERDIR/bin:$PATH
