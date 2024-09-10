export PULP_ENV=/data/project/tsmc65/users/$USER/ws/ddp23/ddp23_pnx
export MY_DDP23_APPS=$PULP_ENV/src/ips/pulpenix_sw/apps
export PULP_GCC_BIN=/data/project/tsmc65/shared/ddp_hackathon/toolchain/pulp-gcc-centos7-20200913/bin
alias ddp23_make="make -f $PULP_ENV/src/tb/sim.make BAUD_RATE=2500000"

