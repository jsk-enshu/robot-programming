// ページごとにセクション番号を1から始めるスクリプト
(function() {
    console.log('Reset section numbers script loaded');

    function resetSectionNumbers() {
        console.log('Resetting section numbers...');

        // H1の番号を非表示にする
        const h1Numbers = document.querySelectorAll('h1 .section-number');
        console.log('Found ' + h1Numbers.length + ' H1 numbers');
        h1Numbers.forEach(function(num) {
            num.style.display = 'none';
        });

        // H2要素を全て取得して番号を再割り当て
        let h2Counter = 0;
        const h2Elements = document.querySelectorAll('article section > h2');
        console.log('Found ' + h2Elements.length + ' H2 elements');

        h2Elements.forEach(function(h2) {
            const numberSpan = h2.querySelector('.section-number');
            if (numberSpan) {
                h2Counter++;
                console.log('Renumbering H2 to ' + h2Counter);
                numberSpan.textContent = h2Counter + '. ';

                // このH2の親sectionを取得
                const parentSection = h2.parentElement;
                if (parentSection) {
                    // 直下の子sectionのH3要素を処理
                    let h3Counter = 0;
                    const childSections = Array.from(parentSection.children).filter(function(el) {
                        return el.tagName === 'SECTION';
                    });

                    childSections.forEach(function(childSection) {
                        const h3 = childSection.querySelector(':scope > h3');
                        if (h3) {
                            const h3NumberSpan = h3.querySelector('.section-number');
                            if (h3NumberSpan) {
                                h3Counter++;
                                console.log('Renumbering H3 to ' + h2Counter + '.' + h3Counter);
                                h3NumberSpan.textContent = h2Counter + '.' + h3Counter + '. ';

                                // H3配下のH4要素を処理
                                let h4Counter = 0;
                                const h3ChildSections = Array.from(childSection.children).filter(function(el) {
                                    return el.tagName === 'SECTION';
                                });

                                h3ChildSections.forEach(function(h4Section) {
                                    const h4 = h4Section.querySelector(':scope > h4');
                                    if (h4) {
                                        const h4NumberSpan = h4.querySelector('.section-number');
                                        if (h4NumberSpan) {
                                            h4Counter++;
                                            console.log('Renumbering H4 to ' + h2Counter + '.' + h3Counter + '.' + h4Counter);
                                            h4NumberSpan.textContent = h2Counter + '.' + h3Counter + '.' + h4Counter + '. ';
                                        }
                                    }
                                });
                            }
                        }
                    });
                }
            }
        });

        // 左サイドバーのすべてのページリンクから番号を削除
        const sidebarLinks = document.querySelectorAll('.bd-sidenav li.toctree-l1 > a');
        console.log('Found ' + sidebarLinks.length + ' links in left sidebar');
        sidebarLinks.forEach(function(link) {
            const originalText = link.textContent;
            console.log('Original link text: "' + originalText + '"');
            // textContentを使って番号を削除
            const newText = link.textContent.replace(/^(\d+)\.\s*/, '');
            if (originalText !== newText) {
                link.textContent = newText;
                console.log('Changed to: "' + newText + '"');
            }
        });

        // 右サイドバー（ページ内目次）の番号だけを更新
        const pageToc = document.querySelector('.bd-toc-nav.page-toc');
        if (!pageToc) {
            console.log('Page TOC not found');
            return;
        }

        // H1レベルの番号を削除
        const tocH1Links = pageToc.querySelectorAll('li.toc-h1 > a');
        console.log('Found ' + tocH1Links.length + ' TOC H1 links in page-toc');
        tocH1Links.forEach(function(link) {
            // 番号パターン "14. " を削除
            const newText = link.innerHTML.replace(/^(\d+)\.\s*/, '');
            link.innerHTML = newText;
            console.log('Removed H1 number from page TOC');
        });

        // H2レベルの番号を書き換え
        const tocH2Links = pageToc.querySelectorAll('li.toc-h2 > a');
        console.log('Found ' + tocH2Links.length + ' TOC H2 links in page-toc');
        let tocH2Counter = 0;

        tocH2Links.forEach(function(link) {
            tocH2Counter++;
            // 番号パターン "14.1. " を "1. " に置き換え
            const newText = link.innerHTML.replace(/^(\d+)\.(\d+)\.\s*/, tocH2Counter + '. ');
            link.innerHTML = newText;
            console.log('Renumbered page TOC H2 to ' + tocH2Counter);
        });

        // H3レベルの番号を書き換え
        const tocH2Items = pageToc.querySelectorAll('li.toc-h2');
        tocH2Items.forEach(function(h2Item, h2Index) {
            let h3Counter = 0;
            const h3Items = h2Item.querySelectorAll(':scope > ul > li.toc-h3');
            h3Items.forEach(function(h3Item) {
                h3Counter++;
                const h3Link = h3Item.querySelector(':scope > a');
                if (h3Link) {
                    // 番号パターン "14.3.1. " を "3.1. " に置き換え
                    const newText = h3Link.innerHTML.replace(/^(\d+)\.(\d+)\.(\d+)\.\s*/, (h2Index + 1) + '.' + h3Counter + '. ');
                    h3Link.innerHTML = newText;
                    console.log('Renumbered page TOC H3 to ' + (h2Index + 1) + '.' + h3Counter);
                }

                // H4レベルの番号を書き換え
                let h4Counter = 0;
                const h4Links = h3Item.querySelectorAll(':scope > ul > li.toc-h4 > a');
                h4Links.forEach(function(h4Link) {
                    h4Counter++;
                    // 番号パターン "14.3.2.1. " を "3.2.1. " に置き換え
                    const newText = h4Link.innerHTML.replace(/^(\d+)\.(\d+)\.(\d+)\.(\d+)\.\s*/, (h2Index + 1) + '.' + h3Counter + '.' + h4Counter + '. ');
                    h4Link.innerHTML = newText;
                    console.log('Renumbered page TOC H4 to ' + (h2Index + 1) + '.' + h3Counter + '.' + h4Counter);
                });
            });
        });

        console.log('Section number reset complete');
    }

    // DOMContentLoadedイベントで実行
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', resetSectionNumbers);
    } else {
        // Already loaded
        resetSectionNumbers();
    }
})();
