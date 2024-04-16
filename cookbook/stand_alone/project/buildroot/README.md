
**HOW TO USE BUILDROOT SCRIPT**
___________________________

1. Choose the name of your app in your `srcs.mk`.
2. You can also fine-tune settings by modifying the top variable in `make_buildroot.sh`.
    - Additionally, adjust proxy settings if necessary (the default configuration is set for use at ENAC/Recherche).
3. From the root directory (where you use `make`):
    - Run `make buildroot`:
        - This command will:
            - Clone Buildroot if necessary.
            - Create a local Buildroot package for your application.
            - (Modify proxy configuration for use at ENAC).
            - Rsync your local package with a Buildroot package.
            - Configure Buildroot for Djnn/Smala and your application.
            - Build your application on Buildroot for the selected board (RPI4 by default).
4. Afterward, you can burn your SD card using `make burn`.
5. Alternatively, update the configuration if it's already running with `make buildroot_update`.
