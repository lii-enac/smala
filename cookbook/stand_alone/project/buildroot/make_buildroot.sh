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

HOME_PROJECT_DIR=`pwd`

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
    echo
    echo -e "${YELLOW}$1${NC}"
}

# Fonction pour afficher un message rouge
error_echo() {
    echo -e "${RED}$1${NC}"
}

check_branch_clone() {
    git ls-remote --exit-code --heads ${LII_GIT_URL} "$1" &>/dev/null
}

# ----- Functions
check_and_clone() {
    if [ -d "${BUILDROOT_DIR}" ]; then
        echo "the directory lii_buildroot_$BOARD exist"
        cd ${BUILDROOT_DIR}
        current_branch=$(git rev-parse --abbrev-ref HEAD)
        if [ "$current_branch" != "$PACKAGE_BRANCH" ]; then
            if git show-ref --verify --quiet "refs/heads/$PACKAGE_BRANCH"; then
                # Si la branche existe, passer simplement dessus
                echo "Switching to branch $PACKAGE_BRANCH"
                git checkout "$PACKAGE_BRANCH"
            else
                # Sinon, créer une nouvelle branche MAIS depuis ${LII_GIT_BRANCH}
                git checkout  ${LII_GIT_BRANCH} 
                warning_echo "Creating branch $PACKAGE_BRANCH from ${LII_GIT_BRANCH}"
                git checkout -b "$PACKAGE_BRANCH"
            fi
        else
            echo "the branch $PACKAGE_BRANCH exist"
        fi
    else
        # le repertoire lii_buildroot_{BOARD} n'existe pas on essai de clone 
        # si la branche existe deja on la recupere
        # sinon on en creer une nouvelle depuis LII_GIT_BRANCH
        if check_branch_clone ${PACKAGE_BRANCH}; then
            COMMANDE="git clone -b "$PACKAGE_BRANCH" $LII_GIT_URL ${BUILDROOT_DIR}"
            info_echo "${COMMANDE}"
            ${COMMANDE}
            warning_echo "Clone lii_buildroot_$BOARD/${PACKAGE_BRANCH}"
        else
            COMMANDE="git clone -b "$LII_GIT_BRANCH" $LII_GIT_URL ${BUILDROOT_DIR}"
            info_echo "${COMMANDE}"
            ${COMMANDE}
            warning_echo "Clone lii_buildroot_$BOARD"
            cd ${BUILDROOT_DIR}
            warning_echo "Create $PACKAGE_BRANCH new branch"
            git checkout -b $PACKAGE_BRANCH
        fi
    fi
}

rsync_local() {
    
    cd ${HOME_PROJECT_DIR}
    rsync -avh project/buildroot/${PACKAGENAME}_Config.in ${BUILDROOT_DIR}/package/Config.in
    if [ $? -ne 0 ]; then
        error_echo "rsync failed."
        exit 0
    fi
    rsync -avh project/buildroot/$PACKAGENAME ${BUILDROOT_DIR}/package/.
    if [ $? -ne 0 ]; then
        error_echo "rsync failed."
        exit 0
    fi
    
    rsync -avh "project/buildroot/${PACKAGE_DEFCONFIG}" "${BUILDROOT_DIR}/."
    if [ $? -ne 0 ]; then
        error_echo "rsync failed."
        exit 0
    fi

    # save configuration
    cd ${BUILDROOT_DIR}

    local repertoire="package/toto"

    info_echo "-- git add"
    # Check if th efile is followed by git
    if ! git ls-files --error-unmatch ${PACKAGE_DEFCONFIG} >/dev/null 2>&1; then
        git add ${PACKAGE_DEFCONFIG}
    fi

    # Check if th efile is followed by git
    if ! git ls-files --error-unmatch package/${PACKAGENAME} >/dev/null 2>&1; then
        git add package/${PACKAGENAME}
    fi

    info_echo "-- git status"
    git status
    info_echo "-- git commit -a "
    git commit -a -m "save ${PACKAGENAME} configuration"
    info_echo "-- git push "
    git push origin ${PACKAGE_BRANCH}:${PACKAGE_BRANCH}

    cd ${HOME_PROJECT_DIR}
}

configure_make() {
    cd "${BUILDROOT_DIR}" || exit 1
    make defconfig BR2_DEFCONFIG="${PACKAGE_DEFCONFIG}"
}

launch_make() {

    warning_echo "Cleaning up buildroot package \"${PACKAGENAME}\" for a new release"
    rm -rf ${BUILDROOT_DIR}/output/build/${PACKAGENAME}*
    rm -rf ${BUILDROOT_DIR}/dl/${PACKAGENAME}
    
    info_echo "Starting compilation..."
    cd ${BUILDROOT_DIR}
    make
}

create_package () {

    cd ${HOME_PROJECT_DIR}/project/buildroot
    ./make_package.sh ${PACKAGENAME} ${DEFAULT_IP}

}

install() {

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
    info_echo "------ 4. RSYNC package : $PACKAGENAME to ${BUILDROOT_DIR}/$PACKAGE_BRANCH"
    echo
    
    rsync_local
    
    echo
    info_echo "------ 5. configure buildroot with : ${BUILDROOT_DIR}/${PACKAGE_DEFCONFIG} "
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
    sudo dd if=${BUILDROOT_DIR}/output/images/sdcard.img of="$device" bs=4M status=progress
}

update() {
    echo
    info_echo "UPDATE with IP=$DEFAULT_IP"

    cd ${HOME_PROJECT_DIR}

    local target_dir="${BUILDROOT_DIR}/output/target"
    local file_to_check="${target_dir}/THIS_IS_NOT_YOUR_ROOT_FILESYSTEM"
    local remote_root="/"
    local remote_user="root"

    if [ -e "${file_to_check}" ]; then
        
        info_echo "killing all application"
        ssh -f ${remote_user}@${DEFAULT_IP} "ka"
        info_echo "cleaning /etc/init.d"
        ssh -f ${remote_user}@${DEFAULT_IP} "rm -rfv /etc/init.d/S30*"
        ssh -f ${remote_user}@${DEFAULT_IP} "rm -rfv /etc/init.d/S45wifi"
        cd ${target_dir}
        info_echo "starting scp from $(pwd) ... to ${remote_user}@${DEFAULT_IP}"
        scp -rp root/ ${remote_user}@${DEFAULT_IP}:${remote_root}
        scp -rp usr/lib/libdjnn* ${remote_user}@${DEFAULT_IP}:${remote_root}/usr/lib/
        scp -rp usr/lib/libsmala* ${remote_user}@${DEFAULT_IP}:${remote_root}/usr/lib/
        scp -rp usr/bin/ka ${remote_user}@${DEFAULT_IP}:${remote_root}/usr/bin/
        scp -rp etc/init.d/S30${PACKAGENAME} ${remote_user}@${DEFAULT_IP}:${remote_root}/etc/init.d/
        scp -rp etc/init.d/S45wifi ${remote_user}@${DEFAULT_IP}:${remote_root}/etc/init.d/
        scp -rp etc/network/interfaces ${remote_user}@${DEFAULT_IP}:${remote_root}/etc/network/
        scp -rp etc/inittab ${remote_user}@${DEFAULT_IP}:${remote_root}/etc/
        scp -rp etc/hostname ${remote_user}@${DEFAULT_IP}:${remote_root}/etc/

    else
        error_echo "you try to scp with the wrong directory : ${target_dir}"
        exit 1
    fi

    info_echo "rebooting for ${PACKAGENAME} application on ${BOARD}"
    ssh -f ${remote_user}@${DEFAULT_IP} "reboot" 

   cd ${HOME_PROJECT_DIR}
   exit 0
}

# ----- Main
case "$1" in
    "install") install ;;
    "burn") burn ;;
    "update") update ;;
    *) error_echo "Usage: ./make_buildroot [install|burn] [package_name]" && exit 1 ;;
esac