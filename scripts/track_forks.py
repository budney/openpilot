#!/usr/bin/env python

import yaml
import json
import git

with open("fork_tracking.yml", "r") as stream:
    try:
        config = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

for module in config['modules']:
    g = git.cmd.Git(config['modules'][module]['path'])
    print('\nModule: ' + module)
    print(g.fetch('--all'))

    for branch in config['modules'][module]['branches']:
        print('Branch: ' + branch['name'])
        try:
            output = g.checkout(branch['name'])
            if output.strip() != '':
                print(output)
        except:
            print('failed to check out branch; skipping')
            continue

        try:
            if branch['type'] == 'pull':
                output = g.pull(branch['source'].split('/'))
                if output.strip() != '':
                    print(output)
            elif branch['type'] == 'merge':
                output = g.merge(branch['source'])
                if output.strip() != '':
                    print(output)
        except:
            print('failed to pull changes from ' + branch['source'] + '. skipping')
            continue

        output = g.push(branch['dest'].split('/'))
        if output.strip() != '':
            print(output)
