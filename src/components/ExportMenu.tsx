import { Button, Menu } from '@mantine/core';
import { useMemo } from 'react';
import { toPrintableObjects } from '../lib/export/printableObjects';
import { downloadStl } from '../lib/export/stl';
import type { Bin } from '../lib/types';

interface Props {
  bins: Bin[];
  generating: boolean;
}

const DOWNLOAD_SPACING_MS = 300;

export function ExportMenu({ bins, generating }: Props) {
  const printables = useMemo(() => toPrintableObjects(bins), [bins]);
  const disabled = generating || printables.length === 0;

  function downloadAll() {
    printables.forEach((printable, index) => {
      setTimeout(() => downloadStl(printable.vertices, printable.name), index * DOWNLOAD_SPACING_MS);
    });
  }

  if (printables.length <= 1) {
    return (
      <Button
        disabled={disabled}
        onClick={() => printables[0] && downloadStl(printables[0].vertices, printables[0].name)}
        title={disabled ? 'Waiting for geometry…' : 'Download STL file'}
      >
        Export STL
      </Button>
    );
  }

  return (
    <Menu>
      <Menu.Target>
        <Button disabled={disabled} rightSection="▾">
          Export STL ({printables.length} parts)
        </Button>
      </Menu.Target>
      <Menu.Dropdown>
        <Menu.Item fw={600} onClick={downloadAll}>
          Download all ({printables.length})
        </Menu.Item>
        <Menu.Divider />
        {printables.map((printable, index) => (
          <Menu.Item
            key={`${printable.name}:${index}`}
            onClick={() => downloadStl(printable.vertices, printable.name)}
          >
            {printable.name}
          </Menu.Item>
        ))}
      </Menu.Dropdown>
    </Menu>
  );
}
