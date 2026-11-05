FROM mediawiki:1.46.2

# Commented out lines are already installed
# RUN git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-Cite /var/www/html/extensions/Cite -b REL1_45
# RUN git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-CodeEditor /var/www/html/extensions/CodeEditor -b REL1_45
# RUN git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-InputBox /var/www/html/extensions/InputBox -b REL1_45
# RUN git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-Gadgets /var/www/html/extensions/Gadgets -b REL1_45
# RUN git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-MultimediaViewer /var/www/html/extensions/MultimediaViewer -b REL1_45
# RUN git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-PageImages /var/www/html/extensions/PageImages -b REL1_45
# RUN git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-ParserFunctions /var/www/html/extensions/ParserFunctions -b REL1_45
# RUN git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-PdfHandler /var/www/html/extensions/PdfHandler -b REL1_45
# RUN git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-Scribunto /var/www/html/extensions/Scribunto -b REL1_45
# RUN git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-SyntaxHighlight_GeSHi /var/www/html/extensions/SyntaxHighlight_GeSHi -b REL1_45
# RUN git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-TemplateData /var/www/html/extensions/TemplateData -b REL1_45
# RUN git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-TextExtracts /var/www/html/extensions/TextExtracts -b REL1_45
# RUN git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-WikiEditor /var/www/html/extensions/WikiEditor -b REL1_45
# RUN git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-TemplateStyles /var/www/html/extensions/TemplateStyles -b REL1_45

# Consolidated version
RUN git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-Citoid /var/www/html/extensions/Citoid -b REL1_45 && \
    git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-CommonsMetadata /var/www/html/extensions/CommonsMetadata -b REL1_45 && \
    git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-ConfirmAccount /var/www/html/extensions/ConfirmAccount -b REL1_45 && \
    git clone --depth 1 https://github.com/jayktaylor/mw-discord /var/www/html/extensions/Discord && \
    # REL1_45 doesn't exist for this extension; 3.7.1 is the most recent release that works with MediaWiki 1.46
    git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-ExternalData /var/www/html/extensions/ExternalData -b 3.7.1 && \
    git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-JsonConfig /var/www/html/extensions/JsonConfig -b REL1_45 && \
    git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-Lockdown /var/www/html/extensions/Lockdown -b REL1_45 && \
    git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-MobileFrontend /var/www/html/extensions/MobileFrontend -b REL1_45 && \
    git clone --depth 1 https://github.com/wikimedia/mediawiki-extensions-Popups /var/www/html/extensions/Popups -b REL1_45 && \
    git clone --depth 1 https://github.com/jmnote/SimpleMathJax /var/www/html/extensions/SimpleMathJax

# Install vim (for easier debugging) and wget
RUN apt-get update && apt-get install -y vim=2:9.1.1230-2+deb13u1 wget=1.25.0-2 --no-install-recommends && \
    apt-get install -y php-common=2:96 php8.4-common=8.4.27-1~deb13u1 --no-install-recommends && \
# Install php-zip extension (for composer)
    apt-get install -y zip=3.0-15+deb13u1 unzip=6.0-29+deb13u1 --no-install-recommends && \
    apt-get install -y php8.4-zip=8.4.27-1~deb13u1 --no-install-recommends && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install composer
RUN wget --progress=dot:giga -cO - https://getcomposer.org/composer-2.phar > /usr/local/bin/composer && \
    chmod +x /usr/local/bin/composer

# Run composer in TemplateStyles to finish installation (https://www.mediawiki.org/wiki/Extension:TemplateStyles#Installation)
# hadolint ignore=DL3003
RUN cd /var/www/html/extensions/TemplateStyles && touch composer.json && composer install --no-dev

# Enable apache2 headers
# Symlink 'w' in /var/www/html to provide cute URLs
RUN a2enmod headers && ln -s /var/www/html /var/www/html/w
# Ensure that w symbolic link is owned by user/group 1000 and ensure that images
# folder is owned by www-data user (which stores cache data)
RUN chown -R 1000:1000 /var/www/html/w && chown -R www-data:www-data /var/www/html/images

# Setup resources
COPY branding/mil_white.svg /var/www/html/resources/assets/mil-white.svg
COPY infra/services/milpedia/LocalSettings.php /var/www/html/LocalSettings.php
COPY infra/services/milpedia/apache.conf /etc/apache2/sites-available/000-default.conf

# Use this to preserve environment variables in LocalSettings.php
# Use arguments JSON notation for CMD and ENTRYPOINT arguments
ENTRYPOINT ["/usr/sbin/apache2ctl", "-D", "BACKGROUND", "&&", "bash"]
