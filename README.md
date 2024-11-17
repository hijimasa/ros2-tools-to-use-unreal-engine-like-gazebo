# ros2-tools-to-use-unreal-engine-like-gazebo

## Prerequisite
1. Docker
1. EpicGamses account

## Prepare Docker container
1. Open your Unreal Engine Account Dashboard

   Sign in to UnrealEngine.com with your verified Epic Games account. Open your account dashboard by clicking on your username and selecting Account from the drop-down menu.

2. Connect to GitHub

   Once the Account Dashboard is open, select the Apps and Accounts tab from the sidebar. Select the Accounts tab, then select the Connect button below the GitHub icon.

3. Connecting your account

   If you have not yet signed the Unreal Engine End User License (EULA), you must read the terms and check the box. Then select Connect Account. When you click the Connect Account button, if you are signed out of your GitHub account, you will be redirected to the GitHub page and asked to sign in.

4. Authorize

   Click the Authorize EpicGames button to complete the OAuth App Authorization process. For more information, see the GitHub description of Authorizing OAuth Apps.

5. Accept the invitation via email

   GitHub will send you an email invitation to join the @EpicGames organization on GitHub. You have 7 days to click the Join @EpicGames button in the email to complete the process of connecting your GitHub account to your Epic Games account.

6. Login ghcr.io with docker

   To retrieve the container image, you will need to log in using the docker login command, as you must have permission to access the Unreal Engine source code.
   At this time, you will be asked for a user and password, so enter the username and password of your GitHub account (or a token with read:package permission).
   ```
   docker login ghcr.io
   ```

7. Clone this repository
   ```
   git clone https://github.com/hijimasa/ros2-tools-to-use-unreal-engine-like-gazebo.git
   ```
8. Initialize Git submodule
   ```
   cd ros2-tools-to-use-unreal-engine-like-gazebo/
   git submodule update --init --recursive
   ```
9. Move to docker directory
   ```
   cd docker
   ```
10. Build docker image
    ```
    ./build_docker_image.sh
    ```
11. Run docker container
    ```
    ./launch_docker.sh
    ```

## New Project Tips
- Set "Edit -> Project Settings -> Maps&Modes -> DefaultMode -> Default GameMode" to "RRROS2GameMode"

## Reference

- [Dockerを使ってUnreal Engineのコンテナイメージを利用する](https://colory-games.net/site/use_unreal_engine_container_image/)
- [GitHub で Unreal Engine のソースコードにアクセス](https://www.unrealengine.com/ja/ue-on-github)
