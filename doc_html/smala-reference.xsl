<?xml version='1.0'?>
<xsl:stylesheet
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  xmlns:fo="http://www.w3.org/1999/XSL/Format"
  xmlns:exsl="http://exslt.org/common"
  version="1.0"
  exclude-result-prefixes="exsl">

  <xsl:import href="../../../docs/djnn.xsl"/>


  <!-- Numbered headings -->
  <xsl:param name="toc.section.depth">1</xsl:param>

  <!-- No chapter number -->
  <xsl:param name="local.l10n.xml" select="document('')"/>
  <l:i18n xmlns:l="http://docbook.sourceforge.net/xmlns/l10n/1.0">
    <l:l10n language="en">
     <l:context name="title-numbered">
       <l:template name="chapter" text="%t"/>
     </l:context>
    </l:l10n>
  </l:i18n>


<xsl:template match="section" mode="toc">
</xsl:template>

</xsl:stylesheet>
