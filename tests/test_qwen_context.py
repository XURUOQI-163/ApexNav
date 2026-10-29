import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch, MagicMock

from llm.answer_reader.answer_reader import read_answer


class QwenContextTests(unittest.TestCase):
    def config(self, policy='read_write'):
        return NS(llm_client='qwen', cache_policy=policy,
                  qwen=NS(model='qwen-plus', base_url='https://example.test/v1', timeout=30))

    def test_cloud_request_and_json(self):
        from llm.answer import get_answer
        sdk = MagicMock()
        sdk.return_value.chat.completions.create.return_value = NS(choices=[NS(message=NS(
            content=json.dumps(dict(confidence=0.45, room='living room'))))])
        with patch.dict('os.environ', {'DASHSCOPE_API_KEY': 'test-secret'}), patch('openai.OpenAI', sdk):
            answer, _ = get_answer(self.config(), 'yellow chair')
        self.assertEqual(answer, [.45, 'living room'])
        request = sdk.return_value.chat.completions.create.call_args.kwargs
        self.assertEqual(request['messages'][-1]['content'], 'yellow chair')
        self.assertEqual(request['model'], 'qwen-plus')
        self.assertEqual(sdk.call_args.kwargs['api_key'], 'test-secret')

    def test_cache_isolation_refresh_off_and_latest(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'answers.txt'
            log = Path(directory)/'responses.txt'
            legacy = "chair: [0.5, 'everywhere']\n"
            path.write_text(legacy)
            with patch('llm.answer_reader.answer_reader.get_answer', return_value=([.45, 'kitchen'], 'raw')) as call:
                read = lambda cfg: read_answer(str(path), str(log), 'chair', cfg)
                self.assertEqual(read(self.config()), ('kitchen', .45))
                read(self.config())
                self.assertEqual(call.call_count, 1)
                call.return_value = ([.55, 'bedroom'], 'new')
                read(self.config('refresh'))
                self.assertEqual(read(self.config()), ('bedroom', .55))
                self.assertEqual(call.call_count, 2)
                before = {p.name: p.read_bytes() for p in Path(directory).iterdir()}
                read(self.config('off'))
                self.assertEqual(before, {p.name: p.read_bytes() for p in Path(directory).iterdir()})
                cfg = self.config(); cfg.qwen.model = 'qwen-turbo'
                read(cfg)
                self.assertEqual(call.call_count, 4)
            self.assertEqual(path.read_text(), legacy)

    def test_invalid_answer_not_cached(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch('llm.answer_reader.answer_reader.get_answer', return_value=([float('nan'), 'room'], 'bad')):
                with self.assertRaises(ValueError):
                    read_answer(directory+'/answer', directory+'/response', 'chair', self.config())
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_missing_key(self):
        from llm.answer import get_answer
        with patch.dict('os.environ', {}, clear=True), self.assertRaisesRegex(ValueError, 'DASHSCOPE_API_KEY'):
            get_answer(self.config(), 'chair')

    def test_deepseek_response_is_not_overwritten(self):
        from llm.answer import get_answer
        stub = NS(deepseek_respond=MagicMock(return_value='Answer: [0.45, "kitchen"]'))
        with patch.dict('sys.modules', {'llm.client.deepseek_answer': stub}):
            answer, raw = get_answer(NS(llm_client='deepseek'), 'chair')
        self.assertEqual(answer, [.45, 'kitchen'])
        self.assertTrue(raw.startswith('Answer:'))

    def test_bad_cloud_json_closes_client(self):
        from llm.answer import get_answer
        sdk = MagicMock()
        sdk.return_value.chat.completions.create.return_value = NS(choices=[NS(message=NS(content='invalid'))])
        with patch.dict('os.environ', {'DASHSCOPE_API_KEY': 'test-secret'}), patch('openai.OpenAI', sdk):
            with self.assertRaises(ValueError):
                get_answer(self.config(), 'chair')
        sdk.return_value.close.assert_called_once()

    def test_obsolete_custom_cache_is_skipped(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'answers.txt'
            path.write_text("chair: ['stool', 0.45, 'kitchen']\n")
            cfg = NS(llm_client='none')
            self.assertEqual(read_answer(str(path), directory+'/responses', 'chair', cfg),
                             ('everywhere', 0.5))
            cfg = NS(llm_client='ollama', ollama='qwen3:8b')
            with patch('llm.answer_reader.answer_reader.get_answer', return_value=([.55, 'living room'], 'raw')) as call:
                self.assertEqual(read_answer(str(path), directory+'/responses', 'chair', cfg),
                                 ('living room', .55))
                read_answer(str(path), directory+'/responses', 'chair', cfg)
                call.assert_called_once()
