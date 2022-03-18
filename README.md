# Neural Stimulator Firmware 2022

This is the repository for the Neural Stimulator Firmware team for 2022.

## Contents:

All of the work of the Firmware team for this year is divided up into two folders:
* The `/firmware` folder contains the actual firmware for the project
* The `/other` folder contains all of the other code created by the team (i.e. weekly tasks)

## Making Changes:

If you wish to make any changes to the repo, please do the following: 
* Make a copy of the repository using `git clone <SSH or HTTPS>`
* Make a new branch for the repo:
  * On **GitHub**: Using the branches, enter your new branch name and click "create"
  * On **Local Terminal**: Enter into your terminal: `git checkout -b <new_branch_name>`
* Checkout your new branch in the terminal by entering `git fetch && git checkout <new_branch_name>`
* Regularly commit your changes by using `git add` and `git commit`
* When you wish to push your local branch changes to your remote branch, use `git push origin <new_branch_name>` 
* When you wish to merge your changes into the `master` branch, visit the branches page and select, "New pull request" [^1]

[^1]: It's not good practice to push anything directly to the `master` branch. By creating pull requests, it allows for somebody else to review your code and also makes it easier to isolate errors (since pull requests often contain multiple commits) 
