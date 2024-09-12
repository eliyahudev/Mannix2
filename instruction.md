# open the sw:
> code src/ips/pulpenix_sw/apps/

in vscode open hello_gpp_xbox.c
write the registers you need for your accelerator
write a dummy input data to the accelerator memory

# open the xbox for the wrapper
> code src/ips/xbox/xbox_xlr_dmy1.sv

this would be your wrapper
read the registers you chose to the controller
read the input from the "addr in" register address

Next step
integrate the exponent unit into the wrapper
