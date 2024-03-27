#!/bin/bash

if [ $# -gt 2 ]; then
    echo "Usage: ./make_buildroot [install|burn|update] [package_name]"
    exit 1
fi

# ----- Project config

DEFAULT_IP="10.42.0.93"
PACKAGENAME=$2
BOARD=RPI4

BUILDROOT_BRANCH="2024.02"
BUILDROOT_DIR=../lii_buildroot_$BOARD

LII_GIT_URL="ssh://git@git.recherche.enac.fr/liiprojects/lii_buildroot.git"
LII_GIT_BRANCH=LII.$BUILDROOT_BRANCH.$BOARD

PACKAGE_BRANCH=$PACKAGENAME.$BUILDROOT_BRANCH.$BOARD
PACKAGE_DEFCONFIG=${PACKAGENAME}.${BUILDROOT_BRANCH}.${BOARD}.defconfig

# ---- debug config

# echo "-- DEBUG --"
# echo $PACKAGENAME
# echo $BOARD
# echo $BUILDROOT_BRANCH
# echo $LII_GIT_BRANCH
# echo $PACKAGE_BRANCH
# exit 0

# ---- sortie

# Définir les couleurs pour les messages
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Fonction pour afficher un message vert
info_echo() {
    echo -e "${GREEN}$1${NC}"
}

# Fonction pour afficher un message jaune
warning_echo() {
    echo -e "${YELLOW}$1${NC}"
}

# Fonction pour afficher un message rouge
error_echo() {
    echo -e "${RED}$1${NC}"
}


# ----- Functions
check_and_clone() {
    if [ -d "$BUILDROOT_DIR" ]; then
        echo "the directory lii_buildroot_$BOARD exist"
        cd $BUILDROOT_DIR
        current_branch=$(git rev-parse --abbrev-ref HEAD)
        if [ "$current_branch" != "$PACKAGE_BRANCH" ]; then
            if git show-ref --verify --quiet "refs/heads/$PACKAGE_BRANCH"; then
                # Si la branche existe, passer simplement dessus
                echo "Switching to branch $PACKAGE_BRANCH"
                git checkout "$PACKAGE_BRANCH"
            else
                # Sinon, créer une nouvelle branche
                echo "Creating branch $PACKAGE_BRANCH"
                git checkout -b "$PACKAGE_BRANCH"
            fi
        else
            echo "the branch $PACKAGE_BRANCH exist"
        fi
    else
        git clone -b "$LII_GIT_BRANCH" --depth 1 $LII_GIT_URL $BUILDROOT_DIR
        echo "Clone lii_buildroot_$BOARD"
        cd $BUILDROOT_DIR
        echo "Create $PACKAGE_BRANCH branch"
        git checkout -b $PACKAGE_BRANCH
    fi
}

rsync_local() {
    
    cd $HOME_PROJECT_DIR
    rsync -avh project/buildroot/${PACKAGENAME}_Config.in $BUILDROOT_DIR/package/Config.in
    if [ $? -ne 0 ]; then
        error_echo "rsync failed."
        exit 0
    fi
    rsync -avh project/buildroot/$PACKAGENAME $BUILDROOT_DIR/package/.
    if [ $? -ne 0 ]; then
        error_echo "rsync failed."
        exit 0
    fi 
}

configure_make() {
    if [ ! -f "$BUILDROOT_DIR/${PACKAGE_DEFCONFIG}" ]; then
        # Le fichier de configuration n'existe pas, nous devons le copier et le configurer
        cd "$HOME_PROJECT_DIR" || exit 1
        rsync -avh "project/buildroot/${PACKAGE_DEFCONFIG}" "$BUILDROOT_DIR/."
        cd "$BUILDROOT_DIR" || exit 1
        make defconfig BR2_DEFCONFIG="${PACKAGE_DEFCONFIG}"
    else
        echo "${PACKAGE_DEFCONFIG} already exists"
        read -p "Would you like to replace it and reconfigure, or you could also use 'xconfig' to modify configuration ? (y/N) " -i N answer
        answer=$(echo "$answer" | tr '[:upper:]' '[:lower:]')
        if [ "$answer" = "y" ] || [ "$answer" = "yes" ]; then
            # Remplacer le fichier de configuration et le configurer à nouveau
            cd "$HOME_PROJECT_DIR" || exit 1
            rsync -avh "project/buildroot/${PACKAGE_DEFCONFIG}" "$BUILDROOT_DIR/."
            cd "$BUILDROOT_DIR" || exit 1
            make defconfig BR2_DEFCONFIG="${PACKAGE_DEFCONFIG}"
        else
            warning_echo "Skipping configuration."
        fi
    fi
}

launch_make() {

    read -p "Does the package ${PACKAGENAME} need to be updated from Git (y/N)? " -i N answer
    answer=$(echo "$answer" | tr '[:upper:]' '[:lower:]')
    if [ "$answer" = "y" ] || [ "$answer" = "yes" ]; then
        warning_echo "Cleaning buildroot from new release ..."
        rm -rf ${BUILDROOT_DIR}/output/build/${PACKAGENAME}*
        rm -rf ${BUILDROOT_DIR}/dl/${PACKAGENAME}
    fi


    read -p "Are you ready to compile (Y/n)? " -i Y answer
    answer=$(echo "$answer" | tr '[:upper:]' '[:lower:]')
    if [ "$answer" = "y" ] || [ "$answer" = "yes" ]; then
        info_echo "Starting compilation..."
    elif [ "$answer" = "n" ] || [ "$answer" = "no" ]; then
        error_echo "Compilation aborted."
        exit 1
    fi

    cd $BUILDROOT_DIR
    make
}

create_package () {

    cd ${HOME_PROJECT_DIR}/project/buildroot
    if [ ! -d ${PACKAGENAME} ]; then
        ./make_package.sh ${PACKAGENAME}
    else
        warning_echo "the directory ${HOME_PROJECT_DIR}/project/buildroot/${PACKAGENAME} already exists"
    fi

}

install() {

    HOME_PROJECT_DIR=`pwd`
    
    echo
    info_echo "------ 1. check and clone buildroot"
    echo

    check_and_clone 
    
    echo
    info_echo "------ 2. CREATE localy package : $PACKAGENAME"
    echo
    
    create_package

    echo
    info_echo "------ 3. configure proxy "
    echo
    
    cd ${HOME_PROJECT_DIR}
    cd ${BUILDROOT_DIR}
    ./LII_install_proxy.sh
    
    echo
    info_echo "------ 4. RSYNC package : $PACKAGENAME to $BUILDROOT_DIR/$PACKAGE_BRANCH"
    echo
    
    rsync_local
    
    echo
    info_echo "------ 5. configure buildroot with : $BUILDROOT_DIR/${PACKAGE_DEFCONFIG} "
    echo
    
    configure_make
    
    echo
    info_echo "------ 6. make buildroot for app : $PACKAGENAME "
    echo

    launch_make
}

burn() {

    # Get list of /dev/sdX devices
    device_list=$(ls /dev/sd* | head -n 1)

    # Ask user which /dev/sdX to write to
    echo
    read -e -p "Which /dev/sdX device do you want to write to? " -i "$device_list" device

    if [ ! -e "$device" ]; then
        error_echo "The device $device does not exist."
        exit 1
    fi

    read -p "Are you sure you want to write to ${device}? (Y/n) " -i Y answer
    if [ "$answer" == "n" ] || [ "$answer" == "N" ]; then
        error_echo "Operation cancelled."
        exit 0
    fi

    info_echo  "choose: ${device}"
    sudo dd if=$BUILDROOT_DIR/output/images/sdcard.img of="$device" bs=4M status=progress
}

update() {
    echo
    info_echo "UPDATE with IP=$DEFAULT_IP"
}

# ----- Main
case "$1" in
    "install")
        install
        ;;
    "burn")
        burn
        ;;
    "update")
        update
        ;;
    *)
        error_echo "Usage: ./make_buildroot [install|burn] [package_name]"
        exit 1
        ;;
esac