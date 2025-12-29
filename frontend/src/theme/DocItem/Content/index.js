import React from "react";
import Content from "@theme-original/DocItem/Content";
import ChapterPersonalization from "@site/src/components/ChapterPersonalization";
import { useDoc } from "@docusaurus/plugin-content-docs/client";

/**
 * Wrapper for DocItem/Content that adds personalization controls
 * This allows users to personalize and translate chapter content
 */
export default function ContentWrapper(props) {
  const { metadata, contentTitle } = useDoc();

  // Get the chapter path from doc metadata
  const chapterPath = metadata?.permalink || "";

  // Get the raw markdown content from the document
  // Note: We pass the rendered content via the component
  const originalContent = props.children?.props?.children || "";

  return (
    <>
      {/* Personalization controls above the content */}
      <ChapterPersonalization
        chapterPath={chapterPath}
        originalContent={typeof originalContent === "string" ? originalContent : ""}
      />

      {/* Original document content */}
      <Content {...props} />
    </>
  );
}
