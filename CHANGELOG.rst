^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rcl_logging_syslog
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-09-09)
------------------
* mergify: do not auto-backport when an explicit backport-* label is set (`#224 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/224>`_)
  The "automatic backport to all supported distribution" rule matched every
  merged PR based on rolling, so a PR labeled backport-lyrical (e.g. `#179 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/179>`_)
  was backported to all distribution branches by the automatic rule firing
  in parallel with the label rule. Exclude PRs carrying any backport-*
  label from the automatic rule so explicit labels select the targets.
  Co-authored-by: Claude Fable 5 <noreply@anthropic.com>
* Update README.md links to reflect new ROS documentation structure (`#179 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/179>`_)
  * Update README.md links to reflect new ROS documentation structure
  * Apply batched suggestions from code review
  Co-authored-by: Daisuke Kato <kato.daisuke429@gmail.com>
  ---------
  Co-authored-by: Tomoya Fujita <tomoya.fujita825@gmail.com>
* skip claude code review for pull requests from forks. (`#215 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/215>`_)
  GitHub runs pull_request workflows from forks with a read-only token,
  no repository secrets and no OIDC token, so claude-code-action cannot
  authenticate and fails with "Could not fetch an OIDC token".
  See https://github.com/fujitatomoya/rcl_logging_syslog/pull/179.
  Switching to pull_request_target is not an option: the action also
  requires the triggering actor to have write access, and bypassing that
  with allowed_non_write_users would let untrusted PR content drive
  Claude with the repository's credentials.
  Skip the job when the PR head is a fork instead, as the former Gemini
  dispatch workflow did. Maintainers can still request a review on such
  a PR by commenting "@claude", which runs claude.yml in the base
  repository with the maintainer as the actor.
* remove gemini actinos. (`#210 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/210>`_)
* use local marketplace plugins because of company rules. (`#205 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/205>`_)
* use official claude code marketplace. (`#200 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/200>`_)
* allow mergifyio to start the claude action app. (`#195 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/195>`_)
* Add claude GitHub actions 1788742709211 (`#186 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/186>`_)
  * "Claude PR Assistant workflow"
  * "Claude Code Review workflow"
* support lyrical luth. (`#183 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/183>`_)
* enable ros2-abi-action. (`#163 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/163>`_)
  * enable ros2-abi-action.
  * upgrade ros2-abi-action.
  * always use ros2-abi-action@latest for now.
  * retrigger CI to pick up fixed ros2-abi-action
  * retrigger CI to pick up ros2-abi-action nounset fix
  ---------
* Bump actions/stale from 10 to 11 in the github-actions group (`#172 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/172>`_)
  Bumps the github-actions group with 1 update: [actions/stale](https://github.com/actions/stale).
  Updates `actions/stale` from 10 to 11
  - [Release notes](https://github.com/actions/stale/releases)
  - [Changelog](https://github.com/actions/stale/blob/main/CHANGELOG.md)
  - [Commits](https://github.com/actions/stale/compare/v10...v11)
  ---
  updated-dependencies:
  - dependency-name: actions/stale
  dependency-version: '11'
  dependency-type: direct:production
  update-type: version-update:semver-major
  dependency-group: github-actions
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* Bump actions/setup-python from 6 to 7 in the github-actions group (`#168 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/168>`_)
  Bumps the github-actions group with 1 update: [actions/setup-python](https://github.com/actions/setup-python).
  Updates `actions/setup-python` from 6 to 7
  - [Release notes](https://github.com/actions/setup-python/releases)
  - [Commits](https://github.com/actions/setup-python/compare/v6...v7)
  ---
  updated-dependencies:
  - dependency-name: actions/setup-python
  dependency-version: '7'
  dependency-type: direct:production
  update-type: version-update:semver-major
  dependency-group: github-actions
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* Bump actions/checkout from 6 to 7 in the github-actions group (`#164 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/164>`_)
  Bumps the github-actions group with 1 update: [actions/checkout](https://github.com/actions/checkout).
  Updates `actions/checkout` from 6 to 7
  - [Release notes](https://github.com/actions/checkout/releases)
  - [Commits](https://github.com/actions/checkout/compare/v6...v7)
  ---
  updated-dependencies:
  - dependency-name: actions/checkout
  dependency-version: '7'
  dependency-type: direct:production
  update-type: version-update:semver-major
  dependency-group: github-actions
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* upgrade CMake version. (`#159 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/159>`_)
* Bump the github-actions group with 4 updates (`#154 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/154>`_)
  Bumps the github-actions group with 4 updates: [actions/checkout](https://github.com/actions/checkout), [actions/setup-python](https://github.com/actions/setup-python), [actions/create-github-app-token](https://github.com/actions/create-github-app-token) and [actions/github-script](https://github.com/actions/github-script).
  Updates `actions/checkout` from 3 to 6
  - [Release notes](https://github.com/actions/checkout/releases)
  - [Commits](https://github.com/actions/checkout/compare/v3...v6)
  Updates `actions/setup-python` from 5 to 6
  - [Release notes](https://github.com/actions/setup-python/releases)
  - [Commits](https://github.com/actions/setup-python/compare/v5...v6)
  Updates `actions/create-github-app-token` from 2.1.1 to 3.2.0
  - [Release notes](https://github.com/actions/create-github-app-token/releases)
  - [Changelog](https://github.com/actions/create-github-app-token/blob/main/CHANGELOG.md)
  - [Commits](https://github.com/actions/create-github-app-token/compare/a8d616148505b5069dccd32f177bb87d7f39123b...bcd2ba49218906704ab6c1aa796996da409d3eb1)
  Updates `actions/github-script` from 7.0.1 to 9.0.0
  - [Release notes](https://github.com/actions/github-script/releases)
  - [Commits](https://github.com/actions/github-script/compare/60a0d83039c74a4aee543508d2ffcb1c3799cdea...3a2844b7e9c422d3c10d287c895573f7108da1b3)
  ---
  updated-dependencies:
  - dependency-name: actions/checkout
  dependency-version: '6'
  dependency-type: direct:production
  update-type: version-update:semver-major
  dependency-group: github-actions
  - dependency-name: actions/setup-python
  dependency-version: '6'
  dependency-type: direct:production
  update-type: version-update:semver-major
  dependency-group: github-actions
  - dependency-name: actions/create-github-app-token
  dependency-version: 3.2.0
  dependency-type: direct:production
  update-type: version-update:semver-major
  dependency-group: github-actions
  - dependency-name: actions/github-script
  dependency-version: 9.0.0
  dependency-type: direct:production
  update-type: version-update:semver-major
  dependency-group: github-actions
  ...
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
* enable dependabot. (`#151 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/151>`_)
* adjust the readme with rcl_logging_implementation. (`#146 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/146>`_)
* Contributors: Daisuke Kato, Tomoya Fujita, dependabot[bot]

0.1.2 (2026-02-20)
------------------
* update doc with rcl_logging_implementation support. (`#138 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/138>`_)
  * update doc with rcl_logging_implementation support.
  * typo fixes and address review comments.
  ---------
* Upgrade github action/run-gemini-cli workflows. (`#136 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/136>`_)
* enable actions/stale to close issues and PRs. (`#131 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/131>`_)
  * enable actions/stale to close issues and PRs.
  * address Copilot review comments.
  ---------
* ROSCon 2025 Singapore Slide Deck. (`#125 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/125>`_)
* add gemini-cli github actions. (`#118 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/118>`_)
* Contributors: Tomoya Fujita

0.1.1 (2025-06-10)
------------------
* enable builtin dictionaries with custom ones. (`#111 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/111>`_)
* Support codespell (`#106 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/106>`_)
  * support codespell github action.
  * spelling fixes by codespell.
  * add empty dictionary.
  ---------
* support kilted kaiju, branch rules and github action. (`#102 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/102>`_)
  * support kilted kaiju, branch rules and github action.
  * Update slide deck icnluding PDF.
  * update overview.html.
  ---------
* support full source build via github workflow. (`#95 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/95>`_)
* fix ROS by the Bay presentation slide. (`#91 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/91>`_)
* cosmetic fix for markdown presentation URLs. (`#88 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/88>`_)
* add slide deck for ROS by the Bay 20250130. (`#85 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/85>`_)
  * add slide deck for ROS by the Bay 20250130.
  * add pdf and html slide decks.
  ---------
* fix nightly workflow. (`#82 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/82>`_)
* add label `skip-backport` to skip the backport to downstream branches. (`#79 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/79>`_)
* make nightly build and workflow to be more generic. (`#76 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/76>`_)
  * remove humble and jazzy nightly workflow files.
  * create generic nightly workflow that can be used for any distro.
  ---------
* add nightly workflow files for each distribution. (`#73 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/73>`_)
* add .mergify/config.yml to automatic backport support. (`#68 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/68>`_)
* remove Iron Irwini since it is already End of Life. (`#65 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/65>`_)
* add github workflows status bars in README. (`#60 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/60>`_)
* github workflow script should be distro agnostic package names. (`#56 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/56>`_)
* enable github workflows to rolling branch. (`#52 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/52>`_)
  * enable github workflows to rolling branch.
  * add exec permission for scripts/github_workflows.sh.
  * enable workflow for other distro branches.
  ---------
* Blank issue enabled for miscellaneous issues. (`#47 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/47>`_)
* add github issue templates. (`#43 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/43>`_)
* add note how to bind the /dev/log socket to the container. (`#29 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/29>`_)
* add ROSCon 2024 LT slide deck. (`#28 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/28>`_)
* test: check if the directory exists before removing. (`#26 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/26>`_)
* rsyslogd omfile permission error. (`#25 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/25>`_)
* support basic colcon test. (`#19 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/19>`_)
  * support basic colcon test.
  * add logging file and contents check via rsyslog test conf.
  * add test section in README.
  ---------
* update README about distro support.
* overview slide deck update.
* cosmetic fixes.
* Update Fluentd_Loki_Grafana.md
  Replace demo video since not working.
* Update Fluentd_Loki_Grafana.md
  Upload fluentd/gragana/loki demo video.
* add Fluentd, Loki and Grafana Tutorial. (`#11 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/11>`_)
  * add Fluentd, Loki and Grafana Tutorial.
  * typo fix.
  ---------
* fix mirror-rolling-to-main.yaml workflow.
* Introduce "/etc/rsyslog.d/ros2-logging.conf". (`#10 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/10>`_)
* add mirror-rolling-to-main.yaml. default branch is now rolling.
* Support syslog facility via environmental variable. (`#7 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/7>`_)
* add logger name and change logger level.
* Keep syslog identity until closelog() is called. (`#5 <https://github.com/fujitatomoya/rcl_logging_syslog/issues/5>`_)
  Fixes: https://github.com/fujitatomoya/rcl_logging_syslog/issues/3
* link the overview html in the README.md.
* add overview slide deck, pdf and html.
* Update README.md
  add ros2 logging forwarded to FluentBit demo video.
* update README.md.
* README cosmetic fix.
* add architecture overview and objective.
* update README.
* add LICENSE file.
* add CONTRIBUTING.md file.
* add CONDEOWNERS file.
* change copy right into correct name.
* 1st commit, basic code but it works with rsyslogd.
* add 1st package.xml file.
* Initial commit
* Contributors: Tomoya Fujita, Tomoya.Fujita
