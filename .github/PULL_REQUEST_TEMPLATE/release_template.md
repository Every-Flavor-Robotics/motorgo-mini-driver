<!--- Title above should be Release X.X.X -->

## Description
<!--- Describe your changes in detail -->

## List of included PRs
<!--- Link all PRs included in this release -->

## Checklist:
<!--- Go over all the following points, and put an `x` in all the boxes that apply. -->
<!--- This PR should NOT be merged until all boxes are checked -->
- [ ] This PR is merging a branch titled release-X.x.x into main
- [ ] `library.json` and `library.properties` have been updated to reflect the release version
- [ ] `platformio.ini` has been updated for all example projects to reflect the release version
- [ ] Example code in documentation matches code in example projects
- [ ] The generated Arduino library ZIP has been tested 
- [ ] All new and existing tests passed
- [ ] A draft release has been created on main. Tag and release name should match the format `vX.x.x`

**Once all checklist items are complete, merge PR with a merge commit**

## Post-Merge Checklist
- [ ] Auto generate release notes and publish release
- [ ] Merge Arduino library into main and publish release in `motorgo-arduino`
- [ ] Confirm that docs deploy successfully for both release version and main
- [ ] Publish release notes in Discord
- [ ] Delete branch
