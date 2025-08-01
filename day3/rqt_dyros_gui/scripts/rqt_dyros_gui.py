#!/usr/bin/env python3

import sys

from rqt_gui.main import Main

def add_arguments(parser):
    group = parser.add_argument_group('Options for rqt_dyros_gui plugin')
    group.add_argument('topic', nargs='?', help='The topic name to subscribe to')

main = Main()
sys.exit(main.main(
    sys.argv,
    standalone = 'rqt_dyros_gui/RqtDyrosPlugin',
    plugin_argument_provider = add_arguments
))