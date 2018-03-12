#!/bin/bash
set -e

java -version
gradle -v

function build {
    gradle clean
    rm -Rf dist
    mkdir -p dist/maven

    version=""
    if [ "$1" != "" ];
    then
        version="-Pin_version=$1"
    fi
    # Build
    gradle -x test -Dmaven.repo.local=dist/maven build publishToMavenLocal $version 
}

function deploy {
    cd dist/
    du
    rm -Rf docs
    mkdir -p docs
    #Extract javadoc
    cd docs
    find ../maven -name "*-javadoc.jar" -exec unzip -o {}  \;
    cd ..
    #Publish maven
    if [ "$1" = "minio" ];
    then
        cd maven 
        mc  cp --recursive $4 . $2
        cd ..
    fi
    #Publish javadoc
    if [ "$1" = "minio" ];
    then
        cd docs
        mc  cp --recursive $4 . $3
        cd ..
    fi
}


$@
