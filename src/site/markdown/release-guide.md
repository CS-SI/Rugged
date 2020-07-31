# Rugged Release Guide

This release guide is largely inspired from [Hipparchus Release
Guide](https://www.hipparchus.org/release-guide.html) and [Orekit Release Guide](https://www.orekit.org/site-orekit-development/release-guide.html). It lists the steps that
have been used in the past to release a new version of Rugged. When in doubt
ask the experts: Sébastien Dinot <sebastien.dinot@csgroup.eu> for website questions
and Luc Maisonobe <luc.maisonobe@csgroup.eu> for everything else.

## Prerequisites

1. Obtain private key of the Rugged Signing Key, key id:
   `9A928E5485FBC44A2A4F9C2E9128808ECB6C9DED`
2. Register for account on OSSRH and associate it with the Rugged project, see:
   https://central.sonatype.org/pages/ossrh-guide.html

If you need help with either, ask on the [development section of the Rugged
forum](https://forum.orekit.org/c/rugged-development/).

Once you have a SonaType OSS account, the corresponding credentials must be set
in the `servers` section of your `$HOME/.m2/settings.xml` file, using an id of
`ossrh`:

    <servers>
      <server>
        <id>ossrh</id>
        <username>the user name to connect to the OSS site</username>
        <password>the encrypted password</password>
      </server>
    </servers>

Use `mvn -ep` to generate an encrypted password.

## Install Graphviz

[Graphviz (dot)](https://graphviz.org) is used to generated diagrams of 
the technical documentation (static site).

## Verify the status of develop branch

Before anything, check on the [continuous integration
site](https://sonar.orekit.org/dashboard?id=org.orekit%3Arugged) that everything is fine on
develop branch:

* All tests pass;
* Code coverage is up to the requirements;
* There are no bugs, vulnerabilities or code smells.

If not, fix the warnings and errors first !

It is also necessary to check on the [Gitlab CI/CD](https://gitlab.orekit.org/orekit/rugged/pipelines)
that everything is fine on develop branch (i.e. all stages are passed).

## Prepare Git branch for release

Release will be performed on a dedicated branch, not directly on master or
develop branch. So a new branch must be created as follows and used for
everything else:

    git branch release-X.Y
    git checkout release-X.Y

## Update maven plugins versions

Release is a good opportunity to update the maven plugin versions. They are all
gathered at one place, in a set of properties in `rugged/pom.xml`, for instance:

    <rugged.spotbugs-maven-plugin.version>4.0.4</rugged.spotbugs-maven-plugin.version>
    <rugged.jacoco-maven-plugin.version>0.8.5</rugged.jacoco-maven-plugin.version>
    <rugged.maven-assembly-plugin.version>3.1.1</rugged.maven-assembly-plugin.version>
    ...

You can find the latest version of the plugins using the search feature at
[http://search.maven.org/#search](http://search.maven.org/#search). The
properties name all follow the pattern `rugged.some-plugin-name.version`, the
plugin name should be used in the web form to check for available versions.

Beware that in some cases, the latest version cannot be used due to
incompatibilities. For example when a plugin was not recently updated and
conflicts appear with newer versions of its dependencies.

Beware also that some plugins use configuration files that may need update too.
This is typically the case with `maven-checkstyle-plugin` and
`spotbugs-maven-plugin`. The `/checkstyle.xml` and `/spotbugs-exclude-filter.xml` files may need to be checked.

Before committing these changes, you have to check that everything works. So
run the following command:

    mvn clean
    LANG=C mvn -Prelease site

If something goes wrong, either fix it by changing the plugin configuration or
roll back to an earlier version of the plugin.

Browse the generated site starting at page `target/site/index.html` and check
that everything is rendered properly. You may also check the contents of the pages.

When everything runs fine and the generated site is OK, then you can commit the
changes:

    git add rugged/pom.xml rugged/checkstyle.xml rugged/spotbugs-exclude-filter.xml
    git commit -m "Updated maven plugins versions."

## Updating changes.xml

Finalize the file `src/changes/changes.xml` file.

The release date and description, which are often only set to `TBD` during
development, must be set to appropriate values. The release date at this step
is only a guess one or two weeks in the future, in order to take into account
the 5 days release vote delay.

Replace the `TBD` description with a text describing the version released:
state if it is a minor or major version, list the major features introduced by
the version etc (see examples in descriptions of former versions).

Commit the `changes.xml` file.

    git add src/changes/changes.xml
    git commit -m "Updated changes.xml for official release."

## Updating documentation

Several files must be updated to take into account the new version:

|            file name                |           usage            |                                     required update                                                    |
|-------------------------------------|----------------------------|--------------------------------------------------------------------------------------------------------|
| `src/site/markdown/index.md`        | site home page             | Update the text about the new features (see changes from **changes.xml**)  |
| `src/site/markdown/downloads.md.vm` | downloads links            | Declare the new versions, don't forget the date                                                        |

Once the files have been updated, commit the changes:

    git add src/site/markdown/*.md
    git commit -m "Updated documentation for the release."

## Change library version number

The `pom.xml` file contains the version number of the library. During
development, this version number has the form `X.Y-SNAPSHOT`. For release, the
`-SNAPSHOT` part must be removed.

Commit the change:

    git add pom.xml
    git commit -m "Dropped -SNAPSHOT in version number for official release."

## Check the JavaDoc

Depending the JDK version (Oracle, OpenJDK, etc), some JavaDoc warnings can be present.
Make sure there is no JavaDoc warnings by running the following command:

    mvn javadoc:javadoc

If possible, run the above command with different JDK versions.

## Tag and sign the git repository

When all previous steps have been performed, the local git repository holds the
final state of the sources and build files for the release. It must be tagged
and the tag must be signed. Note that before the vote is finished, the tag can
only signed with a `-RCx` suffix to denote Release Candidate. The final tag
without the `-RCx` suffix will be put once the vote succeeds, on the same
commit (which will therefore have two tags). Tagging and signing is done using
the following command, with `-RCn` replaced with the Release Candidate number:

    git tag X.Y-RCn -s -u 9A928E5485FBC44A2A4F9C2E9128808ECB6C9DED -m "Release Candidate n for version X.Y."

The tag should be verified using command:

    git tag -v X.Y-RCn

## Pushing the branch and the tag

When the tag is ready, the branch and the tag must be pushed to Gitlab so
everyone can review it:

    git push --tags origin release-X.Y

## Static site (technical documentation)

The static site is generated locally using:

    mvn clean
    LANG=C mvn site

The official site is automatically updated on the hosting platform when work is 
merged into branches `develop`, `release-*` or `master`.

## Generating signed artifacts

When these settings have been set up, generating the artifacts is done by
running the following commands:

    mvn deploy -DskipStagingRepositoryClose=true -Prelease

During the generation, maven will trigger gpg which will ask the user for the
pass phrase to access the signing key. If maven didn’t prompt to you, you have to
add `-Dgpg.passphrase=[passphrase]`

Once the commands ends, log into the SonaType OSS site
[https://oss.sonatype.org/](https://oss.sonatype.org/) and check the staging
repository contains the expected artifacts with associated signatures and
checksums:

- rugged-X.Y.pom
- rugged-X.Y.jar
- rugged-X.Y-sources.jar
- rugged-X.Y-javadoc.jar

The signature and checksum files have similar names with added extensions `.asc`,
`.md5` and `.sha1`.

Sometimes, the deployment to Sonatype OSS site also adds files with double extension
`.asc.md5` and `.asc.sha1`, which are in fact checksum files on a signature file
and serve no purpose and can be deleted.

Remove `rugged-X.Y.source-jar*` since they are duplicates of the
`rugged-X.Y-sources.jar*` artifacts. Then click the “Close” button.

## Update Rugged version in Rugged test website

One edit needs to be made to the Rugged website before calling the vote. Fetch the current code:

    git clone https://gitlab.orekit.org/orekit/website-2015

Switch to `develop` branch and edit `_data/rugged/versions.yml` by adding the new version X.Y 
to the list. 

Once the modification pushed to develop branch, the [test website](https://test.orekit.org/rugged) will be updated

## Calling for the vote

Everything is now ready so the developers and PMC can vote for the release.
Create a post in the [Rugged development category of the forum](https://forum.orekit.org/c/rugged-development/)
with a subject line of the form:

    [VOTE] Releasing Rugged X.Y from release candidate n

and content of the form:

    This is a VOTE in order to release version X.Y of the Rugged library.
    Version X.Y is a maintenance release.


    Highlights in the X.Y release are:
      - feature 1 description
      ...
      - feature n description

    The release candidate n can be found on the GitLab repository as
    tag X.Y-RCn in the release-X.Y branch:
    <https://gitlab.orekit.org/orekit/rugged/tree/X.Y-RCn>

    The release notes can be read here:
    <https://test.orekit.org/site-rugged-X.Y/changes-report.html>

    Maven artifacts are available at
    <https://oss.sonatype.org/content/repositories/orgorekit-xxxx/>

    The votes will be tallied in 120 hours for now, on 20yy-mm-ddThh:mm:00Z
    (this is UTC time).

You should also ping PMC members so they are aware of the vote. Their
vote is essential for a release as per project governance.

## Failed vote

If the vote fails, the maven artifacts must be removed from OSS site by
dropping the repository. Then a new release candidate must
be created, with a new number, a new tag and new artifacts. Another vote is
needed for this new release candidate. So make the necessary changes and then
start from the “Tag and sign the git repository” step.

## Successful vote

When the vote for a release candidate succeeds, follow the steps below to
publish the release.

## Tag release version

As the vote passed, a final signed tag must be added to the succeeding release
candidate, verified and pushed:

    git tag X.Y -s -u 9A928E5485FBC44A2A4F9C2E9128808ECB6C9DED -m "Version X.Y."
    git tag -v X.Y
    git push --tags

## Merge release branch into master

Merge the release branch into the `master` branch to include any changes made.

    git checkout master
    git merge --no-ff release-X.Y

Then commit and push.

## Merge master branch into develop

Merge the `master` branch into the `develop` branch to include any changes made.

    git checkout develop
    git merge --no-ff master

Then update the version number to prepare for the next development cycle:

- edit the pom.xml to update version to a SNAPSHOT,
- make space in the `/src/changes/changes.xml` file for new changes. 

Then commit and push.

## Publish maven artifacts

The maven artifacts must be published using OSS site to release the repository.
Select the Rugged repository in "Staging Repositories" and click the “Release”
button in [Nexus Repository Manager](https://oss.sonatype.org/).

## Upload to Gitlab

Navigate to Projects > Rugged > Repository > Tags. Find the X.Y tag and
click the edit button to enter release notes. Use the **path** in the [Nexus 
repository](https://packages.orekit.org/#browse/browse:maven-releases:org%2Forekit%2Frugged) to
set the artifacts in the release notes.

- rugged-X.Y.jar
- rugged-X.Y-sources.jar
- rugged-X.Y-javadoc.jar

Navigate to Projects > Rugged > Project Overview > Releases and make sure it looks nice.

## Update Rugged website

Several edits need to be made to the Rugged website after the vote.

Edit `download/.htaccess` and replace the URLs of the 3 Rugged artifacts
with the ones used to create the release notes.

Edit `_layouts/home_rugged.html` and edit the text of the button to use the new version.

Edit `rugged/overview.html` with the new Orekit and Hipparchus versions. Don't forget to update the
rugged/img/rugged-architecture.png image with the new dependencies.

Create a new post for the release in `_post/` using as template a previous Rugged post (in order to be published in the Rugged News page). To be noticed the `--future` option is needed to see a post in the future, otherwise `jekyll serve` will ignore it !

Run:

    jekyll serve --future

and make sure the website looks nice. View it on http://localhost:4000/

## Close X.Y milestone

In Gitlab, navigate to Projects > Rugged > Issues > Milestones.
Click “Close Milestone” for the line corresponding to the release X.Y.

## Announce release

The last step is to announce the release by creating a post in the 
[Rugged announcements category of the forum](https://forum.orekit.org/c/rugged-announcements/) 
with a subject line of the form:

    Rugged X.Y released

and content of the form:

    The Rugged team is pleased to announce the release of Rugged version X.Y. 
    This is a minor/major version, including both new features and bug fixes. 
    The main changes are:

      - feature 1 description
      ...
      - feature n description

    This version depends on Orekit X.x and Hipparchus Y.y.

    For complete release notes please see:
    https://www.orekit.org/site-rugged-X.Y/changes-report.html

    The maven artifacts are available in maven central. 
    The source and binaries can be retrieved from the forge releases page:
    https://gitlab.orekit.org/orekit/rugged/-/releases
