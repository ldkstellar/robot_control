zsh -c "source ./install/setup.zsh"
projectDir="./src/action_control_cpp/package.xml"
result=$(grep "name" $projectDir)
value=$(echo "$result" | grep -oP '(?<=<name>).*?(?=</name>)')
zsh -c "ros2 run $value ControlActionClient"