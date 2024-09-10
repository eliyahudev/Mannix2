
First time setup (just once)  
----------------------------

open a terminal and enter following commands   

tsmc65  

mkdir  /data/project/tsmc65/users/$USER/ws/ddp23  

cd  /data/project/tsmc65/users/$USER/ws/ddp23  


 

git clone  https://gitlab.com/udik/ddp23_pnx.git  
gitlab username and password might be required  

Only in case above clone does not work try this:  
git clone git@gitlab.com:udik/ddp23_pnx.git  

mkdir sim  
  
Edit ~/.bashrc  (This file automatically runs every time you open a terminal)  
You may invoke editing the file with vscode by  
code ~/.bashrc  
Carefully add this line to the end of the file:  

source /data/project/tsmc65/users/$USER/ws/ddp23/ddp23_pnx/cloud_setup.sh  

Save and close the file  
close all terminal and open a new one to proceed.  


Helloworld Simulation
---------------------

In a fresh new terminal :  

tsmc65  

cd $PULP_ENV/../sim  

Execute this:

ddp23_make APP=helloworld  

Some warnnings may show up, you may ignore them for now, eventuelly this should be displayed:  

HELLO DDP24  
--- FINISH ---  

If so you are done with the initial setup  

you may now slightly modify the C source file (e.g. personalize to hello your name):  

The C source file is located at:  

$MY_DDP23_APPS/helloworld/helloworld.c   

and re-run:

ddp23_make APP=helloworld

Your modification should be noticed in the outcome simulation print.

# Mannix2
