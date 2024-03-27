#!/bin/bash

# Définition des remplacements
PACKAGENAME=$1
PACKAGENAME_CAP=$(echo "$PACKAGENAME" | tr '[:lower:]' '[:upper:]')

PATTERN_RADICAL=template
PATTERN="@${PATTERN_RADICAL}@"
PATTERN_CAP=$(echo "$PATTERN" | tr '[:lower:]' '[:upper:]')
PATTERN_GIT_URL="@${PATTERN_RADICAL}_git_url@"
PATTERN_GIT_BRANCH="@${PATTERN_RADICAL}_git_branch@"

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
    
    local file="$1"
    if [ "$(uname)" == "Linux" ]; then
        SED_I_OPTION=
    else
        #on Darwin need to be defined at ''
        SED_I_OPTION="''"
    fi
    sed -i ${SED_I_OPTION} -e "s/${PATTERN}/${PACKAGENAME}/g" -e "s/${PATTERN_CAP}/${PACKAGENAME_CAP}/g"  -e "s|${PATTERN_GIT_URL}|${GIT_URL}|g" -e "s|${PATTERN_GIT_BRANCH}|${GIT_BRANCH}|g" "$file"
}

# Répertoire à scanner
directory="@template@"

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

filename=${PATTERN}_Config.in
new_filename=$(echo "$filename" | sed -e "s/${PATTERN}/${PACKAGENAME}/g" -e "s/${PATTERN_CAP}/${PACKAGENAME_CAP}/g")       
cp "$filename" "$new_filename"
echo "File copied: $filename -> $new_filename"
replace_strings "$new_filename"
echo "Replacements done in $new_filename"

filename=${PATTERN}.2024.02.RPI4.defconfig
new_filename=$(echo "$filename" | sed -e "s/${PATTERN}/${PACKAGENAME}/g" -e "s/${PATTERN_CAP}/${PACKAGENAME_CAP}/g")       
cp "$filename" "$new_filename"
echo "File copied: $filename -> $new_filename"
replace_strings "$new_filename"
echo "Replacements done in $new_filename"
