from pathlib import Path

import pytest
from receptionist_bot.receptionist_bot.ReceptionistNode import ReceptionistNode
import yaml

# read the script from the source tree, so the tests check the current file, not the installed one
SCRIPT_PATH = Path(__file__).parent.parent / 'dialog_scripts' / 'short_test_script.yaml'


@pytest.fixture
def node():
    # skip __init__: no rclpy.init(), no .env, no action clients
    return ReceptionistNode.__new__(ReceptionistNode)


@pytest.fixture(scope='module')
def messages():
    with SCRIPT_PATH.open('r', encoding='utf-8') as file:
        return yaml.safe_load(file)['messages']


def schema_of(tool):
    return tool['toolSpec']['inputSchema']['json']


def test_transitions_become_enum_with_unclear(node):
    # Arrange
    message = {
        'transitions': [
            {'if': 'ja', 'goto': 'greet', 'description': 'Besucher hat einen Termin'},
            {'if': 'nein', 'goto': 'bye', 'description': 'Kein Termin'},
        ]
    }

    # Act
    extract_ids, transitions, tool = node._define_Tool(message)

    # Assert
    schema = tool['toolSpec']['inputSchema']['json']
    assert transitions == ['ja', 'nein', 'unclear']
    assert schema['properties']['choice']['enum'] == ['ja', 'nein', 'unclear']
    assert schema['required'] == ['choice']
    assert extract_ids == []


def test_ask_name(node, messages):
    extract_ids, transitions, tool = node._define_Tool(messages['ask_name'])

    schema = schema_of(tool)
    assert extract_ids == ['name']
    assert transitions == ['name_given', 'unclear']
    assert schema['required'] == ['name', 'choice']
    assert schema['properties']['name'] == {
        'type': 'string',
        'description': 'Der Name des Besuchers, so wie er ihn genannt hat',
    }
    assert schema['properties']['choice']['enum'] == ['name_given', 'unclear']
    assert 'name_given: Der Besucher hat einen Namen genannt' in \
        schema['properties']['choice']['description']


def test_ask_professor(node, messages):
    # called like _execute_callback does it: without extract_options
    extract_ids, transitions, tool = node._define_Tool(messages['ask_professor'])

    schema = schema_of(tool)
    assert extract_ids == ['professor']
    assert transitions == ['professor_given', 'unclear']
    assert schema['required'] == ['professor', 'choice']
    # 'options: professors' in the yaml is not read by _define_Tool
    assert 'options' not in schema['properties']['professor']
    assert schema['properties']['choice']['enum'] == ['professor_given', 'unclear']


def test_drive_to_professor(node, messages):
    # drive message: no extract and no transitions, so the tool asks for nothing
    extract_ids, transitions, tool = node._define_Tool(messages['drive_to_professor'])

    assert extract_ids == []
    assert transitions == []
    assert schema_of(tool)['properties'] == {}
    assert schema_of(tool)['required'] == []


def test_drive_failed(node, messages):
    extract_ids, transitions, tool = node._define_Tool(messages['drive_failed'])

    assert extract_ids == []
    assert transitions == []
    assert schema_of(tool)['properties'] == {}
    assert schema_of(tool)['required'] == []


def test_ask_talk(node, messages):
    extract_ids, transitions, tool = node._define_Tool(messages['ask_talk'])

    schema = schema_of(tool)
    assert extract_ids == []
    # must stay strings: unquoted yes/no in the yaml would be loaded as True/False
    assert transitions == ['yes', 'no', 'unclear']
    assert schema['required'] == ['choice']
    assert schema['properties']['choice']['enum'] == ['yes', 'no', 'unclear']


def test_reply_yes(node, messages):
    extract_ids, transitions, tool = node._define_Tool(messages['reply_yes'])

    assert extract_ids == []
    assert transitions == []
    assert schema_of(tool)['properties'] == {}
    assert schema_of(tool)['required'] == []


def test_reply_no(node, messages):
    extract_ids, transitions, tool = node._define_Tool(messages['reply_no'])

    assert extract_ids == []
    assert transitions == []
    assert schema_of(tool)['properties'] == {}
    assert schema_of(tool)['required'] == []
