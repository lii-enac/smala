#!/bin/bash

# Définition des remplacements
PACKAGENAME=$1
PACKAGENAME_CAP=$(echo "$PACKAGENAME" | tr '[:lower:]' '[:upper:]')

PACKAGENAME_IP=$2

PATTERN_RADICAL=template
PATTERN="@${PATTERN_RADICAL}@"
PATTERN_CAP=$(echo "$PATTERN" | tr '[:lower:]' '[:upper:]')
PATTERN_GIT_URL="@${PATTERN_RADICAL}_git_url@"
PATTERN_GIT_BRANCH="@${PATTERN_RADICAL}_git_branch@"
PATTERN_IP="@${PATTERN_RADICAL}_ip@"

#Obtenir l'URL du dépôt Git
GIT_URL=$(git remote get-url origin)
# Obtenir la branche courante
GIT_BRANCH=$(git rev-parse --abbrev-ref HEAD)

echo "packagename : ${PACKAGENAME}"
#echo "packagename_cap : ${PACKAGENAME_CAP}"
echo "pattern : ${PATTERN}"
#echo "pattern_cap : ${PATTERN_CAP}"
echo "git/branch = ${GIT_URL}/${GIT_BRANCH}"
echo

# Fonction pour effectuer les remplacements dans un fichier
replace_strings() {
    
    # note :
    # I have to use a temporary file to be compatible between 'sed' on Linux and Darwin.
    # alaso I have to keep the permissions
    local file="$1"
    local permissions=$(stat -c "%a" "$file")
    local tmp_file=$(mktemp) #tmp_file unique

    sed -e "s/${PATTERN}/${PACKAGENAME}/g" \
        -e "s/${PATTERN_CAP}/${PACKAGENAME_CAP}/g" \
        -e "s|${PATTERN_GIT_URL}|${GIT_URL}|g" \
        -e "s|${PATTERN_GIT_BRANCH}|${GIT_BRANCH}|g" \
        -e "s|${PATTERN_IP}|${PACKAGENAME_IP=$2}|g" \
        "$file" > ${tmp_file}

    chmod ${permissions} ${tmp_file}
    mv ${tmp_file} ${file}
}

# Répertoire à scanner
directory=${PATTERN}

rm -rf "${PACKAGENAME}"
mkdir -p "${PACKAGENAME}"
DEST_DIR="${PACKAGENAME}"

# Parcours des fichiers dans le répertoire
for file in "$directory"/*; do
    if [ -f "$file" ]; then
        # Extraire le nom du fichier
        filename=$(basename "$file")
        # Remplacer les chaînes dans le nom du fichier
        new_filename=$(echo "$filename" | sed -e "s/${PATTERN}/${PACKAGENAME}/g" -e "s/${PATTERN_CAP}/${PACKAGENAME_CAP}/g")       
        cp "$file" "$DEST_DIR/$new_filename"
        echo "File copied: $file -> $DEST_DIR/$new_filename"
        replace_strings "$DEST_DIR/$new_filename"
        echo "Replacements done in $DEST_DIR/$new_filename"
    fi
done

files=("${PATTERN}_Config.in" "${PATTERN}.2024.02.RPI4.defconfig")

for filename in "${files[@]}"; do
    new_filename=$(echo "$filename" | sed -e "s/${PATTERN}/${PACKAGENAME}/g" -e "s/${PATTERN_CAP}/${PACKAGENAME_CAP}/g")
    cp "$filename" "$new_filename"
    echo "File copied: $filename -> $new_filename"
    replace_strings "$new_filename"
    echo "Replacements done in $new_filename"
done
