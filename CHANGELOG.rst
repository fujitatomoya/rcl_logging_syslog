^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rcl_logging_syslog
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
