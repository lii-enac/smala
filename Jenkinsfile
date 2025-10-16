pipeline {
    agent none

    options {
        buildDiscarder(logRotator(numToKeepStr: '10'))
        disableConcurrentBuilds()
        timeout(time: 10, unit: 'MINUTES')
        timestamps()
    }

    triggers {
        pollSCM('H/5 * * * *')
    }

    stages {
        // === macOS build first ===
        stage('Build macOS') {
            agent { label 'macos12' }
            environment {
                CUSTOM_WORKSPACE = '../djnn-qt/smala'
            }
            steps {
                dir(env.CUSTOM_WORKSPACE) {
                    cleanWs()
                    checkout([
                        $class: 'GitSCM',
                        branches: [[name: '*/master']],
                        userRemoteConfigs: [[
                            url: env.SMALA_URL,
                            credentialsId: env.GIT_CREDENTIAL_ID
                        ]]
                    ])

                    sh '''
                        echo "=== Building smala (macOS) ==="
                        echo "djnn-cpp sibling: $(ls -d ../djnn-cpp || echo 'MISSING!')"

                        if [ ! -d "../djnn-cpp" ]; then
                            echo "❌ Error: Missing ../djnn-cpp directory."
                            echo "Make sure djnn-cpp pipeline ran successfully first."
                            exit 1
                        fi

                        export PATH="/opt/homebrew/bin:/opt/homebrew/sbin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin"
                        make -j
                        make -j lib
                        make -j cookbook_apps
                    '''
                }
            }
            post {
                failure {
                    emailext(
                        to: "${env.EMAIL1}, ${env.EMAIL2}",
                        subject: "❌ macOS build failed - smala #${currentBuild.number}",
                        body: "The macOS build failed.\nDetails: ${env.BUILD_URL}\nError (last 100 lines):\n${currentBuild.rawBuild.getLog(100).join('\n')}"
                    )
                }
            }
        }

        // === Ubuntu & Windows parallel builds ===
        stage('Build Ubuntu & Windows') {
            when {
                expression { currentBuild.result == null || currentBuild.result == 'SUCCESS' }
            }
            parallel {
                stage('Build Ubuntu') {
                    agent { label 'ubuntu' }
                    environment {
                        CUSTOM_WORKSPACE = '../djnn-qt/smala'
                    }
                    steps {
                        dir(env.CUSTOM_WORKSPACE) {
                            cleanWs()
                            checkout([
                                $class: 'GitSCM',
                                branches: [[name: '*/master']],
                                userRemoteConfigs: [[
                                    url: env.SMALA_URL,
                                    credentialsId: env.GIT_CREDENTIAL_ID
                                ]]
                            ])
                            sh '''
                                echo "=== Building smala (Ubuntu) ==="
                                echo "djnn-cpp sibling: $(ls -d ../djnn-cpp || echo 'MISSING!')"

                                if [ ! -d "../djnn-cpp" ]; then
                                    echo "❌ Error: Missing ../djnn-cpp directory."
                                    echo "Make sure djnn-cpp pipeline ran successfully first."
                                    exit 1
                                fi

                                make -j8
                                make -j8 lib
                                make -j8 cookbook_apps
                            '''
                        }
                    }
                    post {
                        failure {
                            emailext(
                                to: "${env.EMAIL1}, ${env.EMAIL2}",
                                subject: "❌ Ubuntu build failed - smala #${currentBuild.number}",
                                body: "The Ubuntu build failed.\nDetails: ${env.BUILD_URL}\nError (last 100 lines):\n${currentBuild.rawBuild.getLog(100).join('\n')}"
                            )
                        }
                    }
                }

                stage('Build Windows') {
                    agent { label 'win10' }
                    environment {
                        CUSTOM_WORKSPACE = '../djnn-qt/smala'
                    }
                    steps {
                        dir(env.CUSTOM_WORKSPACE) {
                            cleanWs()
                            checkout([
                                $class: 'GitSCM',
                                branches: [[name: '*/master']],
                                userRemoteConfigs: [[
                                    url: env.SMALA_URL,
                                    credentialsId: env.GIT_CREDENTIAL_ID
                                ]]
                            ])
                            bat '''
                                echo === Building smala (Windows) ===
                                if not exist ..\\djnn-cpp (
                                    echo ❌ Error: Missing ..\\djnn-cpp directory.
                                    echo Make sure djnn-cpp pipeline ran successfully first.
                                    exit /b 1
                                )
                                make -j
                                make -j lib
                                make -j cookbook_apps
                            '''
                        }
                    }
                    post {
                        failure {
                            emailext(
                                to: "${env.EMAIL1}, ${env.EMAIL2}",
                                subject: "❌ Windows build failed - smala #${currentBuild.number}",
                                body: "The Windows build failed.\nDetails: ${env.BUILD_URL}\nError (last 100 lines):\n${currentBuild.rawBuild.getLog(100).join('\n')}"
                            )
                        }
                    }
                }
            }
        }
    }

    post {
        success {
            script {
                if (currentBuild.previousBuild?.result == 'FAILURE') {
                    emailext(
                        to: "${env.EMAIL1}, ${env.EMAIL2}",
                        subject: "✅ Back to normal - smala #${currentBuild.number}",
                        body: """
The build has recovered after a failure.

Previous build: #${currentBuild.previousBuild.number} (${currentBuild.previousBuild.result})
Current build: SUCCESS

Details: ${env.BUILD_URL}
"""
                    )
                }
            }
        }

        failure {
            emailext(
                to: "${env.EMAIL1}, ${env.EMAIL2}",
                subject: "❌ Pipeline build failed - smala #${currentBuild.number}",
                body: """
The build has failed.

Details: ${env.BUILD_URL}

Error (last 100 lines):
${currentBuild.rawBuild.getLog(100).join('\n')}
"""
            )
        }
    }
}
