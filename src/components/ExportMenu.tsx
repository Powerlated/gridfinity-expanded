import { Button, Group, Menu, Text } from '@mantine/core';
import { useMemo, useState } from 'react';
import { buildBinParameters } from '../lib/binParameters';
import { designFilename, toPrintableObjects } from '../lib/export/printableObjects';
import { downloadParasolid, exportParasolid } from '../lib/export/parasolid';
import { downloadStl } from '../lib/export/stl';
import type { Bin, Design } from '../lib/types';

interface Props {
  design: Design;
  bins: Bin[];
  generating: boolean;
}

const DOWNLOAD_SPACING_MS = 300;

export function ExportMenu({ design, bins, generating }: Props) {
  const printables = useMemo(() => toPrintableObjects(bins), [bins]);
  const disabled = generating || printables.length === 0;
  const [exportingXt, setExportingXt] = useState(false);
  const [xtError, setXtError] = useState<string | null>(null);

  async function downloadXt() {
    setExportingXt(true);
    setXtError(null);
    try {
      const xt = await exportParasolid(buildBinParameters(design));
      downloadParasolid(xt, designFilename());
    } catch (error) {
      setXtError(error instanceof Error ? error.message : String(error));
    } finally {
      setExportingXt(false);
    }
  }

  function downloadAll() {
    printables.forEach((printable, index) => {
      setTimeout(() => downloadStl(printable.vertices, printable.name), index * DOWNLOAD_SPACING_MS);
    });
  }

  const stlControl = printables.length <= 1 ? (
    <Button
      disabled={disabled}
      onClick={() => printables[0] && downloadStl(printables[0].vertices, printables[0].name)}
      title={disabled ? 'Waiting for geometry…' : 'Download STL file'}
    >
      Export STL
    </Button>
  ) : (
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

  return (
    <Group gap="xs" align="center" wrap="nowrap">
      {xtError && (
        <Text size="xs" c="red" style={{ maxWidth: 260 }} lineClamp={2}>
          {xtError}
        </Text>
      )}
      {stlControl}
      <Button
        disabled={disabled}
        loading={exportingXt}
        onClick={downloadXt}
        title="Download Parasolid X_T file (analytic B-rep, one body per piece)"
      >
        Export X_T
      </Button>
    </Group>
  );
}
