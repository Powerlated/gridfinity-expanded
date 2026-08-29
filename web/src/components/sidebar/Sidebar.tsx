import { useState } from 'react';
import { ScrollArea, Tabs } from '@mantine/core';
import { ShapeTab } from './tabs/ShapeTab';
import { WallsTab } from './tabs/WallsTab';
import { CutsTab } from './tabs/CutsTab';

/**
 * The left sidebar's tabs: the spatial cell, wall, and cut editors.
 * Form-shaped parameter groups live in `SECTIONS` in SettingsPanel.tsx.
 */
const TABS = {
  Shape: ShapeTab,
  Walls: WallsTab,
  Cuts: CutsTab,
} as const;

type Tab = keyof typeof TABS;

export function Sidebar() {
  const [activeTab, setActiveTab] = useState<Tab>('Shape');
  const ActivePanel = TABS[activeTab];

  return (
    <Tabs
      value={activeTab}
      onChange={(value) => value && setActiveTab(value as Tab)}
      className="sidebar-tabs"
    >
      <Tabs.List>
        {(Object.keys(TABS) as Tab[]).map((tab) => (
          <Tabs.Tab key={tab} value={tab}>
            {tab}
          </Tabs.Tab>
        ))}
      </Tabs.List>
      <Tabs.Panel value={activeTab} flex={1} style={{ minHeight: 0 }}>
        <ScrollArea h="100%" p="md">
          <ActivePanel />
        </ScrollArea>
      </Tabs.Panel>
    </Tabs>
  );
}
